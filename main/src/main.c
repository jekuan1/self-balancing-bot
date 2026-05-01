#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "control/pid_controller.h"
#include "hal/imu_module.h"
#include "hal/motor_module.h"
#include "hal/ota_module.h"
#include "hal/wifi_module.h"
#include "robot_control.h"
#include "supervisor/command_parser.h"

static const char *TAG = "motor_test";

#define ENABLE_OTA_AP 0

typedef struct {
    imu_module_t imu;
    motor_module_t motor_hal;
    motor_command_t test_cmd;
    QueueHandle_t imu_queue;
    QueueHandle_t control_cmd_queue;
    command_parser_t command_parser;
    pid_controller_t balance_pid;
    float target_pitch_deg;
} robot_runtime_t;

static robot_runtime_t g_runtime = {0};

bool robot_control_send_command(robot_control_command_t command)
{
    if (g_runtime.control_cmd_queue == NULL) {
        return false;
    }
    return xQueueOverwrite(g_runtime.control_cmd_queue, &command) == pdTRUE;
}

bool robot_control_send_stop(void)
{
    return robot_control_send_command(ROBOT_CONTROL_CMD_STOP);
}

bool robot_control_send_start(void)
{
    return robot_control_send_command(ROBOT_CONTROL_CMD_START);
}

static void maybe_start_ota_services(void)
{
#if ENABLE_OTA_AP
    esp_err_t wifi_err = wifi_module_init_ap("BalanceBot", "balance123");
    if (wifi_err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi AP init failed: %s", esp_err_to_name(wifi_err));
        return;
    }

    esp_err_t ota_err = ota_module_init();
    if (ota_err != ESP_OK) {
        ESP_LOGE(TAG, "OTA server init failed: %s", esp_err_to_name(ota_err));
        return;
    }
    ESP_LOGI(TAG, "OTA AP mode enabled");
#endif
}

// TMC2240 SPI test wiring (adjust to your board).
#define TMC_SPI_HOST        1
#define TMC_SPI_SCLK        GPIO_NUM_16
#define TMC_SPI_MOSI        GPIO_NUM_15
#define TMC_SPI_MISO        GPIO_NUM_10
#define TMC_EN              GPIO_NUM_18

// ============== LEFT MOTOR PINS ==============

#define TMC_LEFT_STEP       GPIO_NUM_8
#define TMC_LEFT_DIR        GPIO_NUM_13
#define TMC_LEFT_SPI_CS     GPIO_NUM_21
#define TMC_LEFT_DIAG0      GPIO_NUM_6

// ============== RIGHT MOTOR PINS ==============
#define TMC_RIGHT_STEP      GPIO_NUM_2
#define TMC_RIGHT_DIR       GPIO_NUM_1
#define TMC_RIGHT_SPI_CS    GPIO_NUM_39
#define TMC_RIGHT_DIAG0     GPIO_NUM_41

static void imu_task_fn(void *arg)
{
    robot_runtime_t *runtime = (robot_runtime_t *)arg;
    if (runtime == NULL || runtime->imu_queue == NULL) {
        vTaskDelete(NULL);
        return;
    }

    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        if (runtime->imu.i2c_dev != NULL) {
            // Poll the IMU. In the future, this can be triggered by a hardware INT pin.
            imu_module_poll_and_log(&runtime->imu);
            
            imu_sample_t sample;
            if (imu_module_read_sample(&runtime->imu, &sample) == ESP_OK) {
                // Send the sample to the control task, overwriting if queue is full
                xQueueOverwrite(runtime->imu_queue, &sample);
            }
        }
        // Poll at 100Hz
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}

static void control_task_fn(void *arg)
{
    robot_runtime_t *runtime = (robot_runtime_t *)arg;
    if (runtime == NULL || runtime->imu_queue == NULL) {
        vTaskDelete(NULL);
        return;
    }

    motor_module_set_enabled(&runtime->motor_hal, true);
    ESP_LOGI(TAG, "Control Task Started. Waiting for IMU data...");

    // Pitch polarity can be flipped if IMU mounting orientation is inverted.
    const float PITCH_POLARITY = -1.0f;
    bool stopped = false;

    while (1) {
        robot_control_command_t command = ROBOT_CONTROL_CMD_NONE;
        if (runtime->control_cmd_queue != NULL && xQueueReceive(runtime->control_cmd_queue, &command, 0) == pdTRUE) {
            if (command == ROBOT_CONTROL_CMD_STOP) {
                runtime->test_cmd.left_step_hz = 0.0f;
                runtime->test_cmd.right_step_hz = 0.0f;
                motor_module_apply_command(&runtime->motor_hal, &runtime->test_cmd);
                motor_module_set_enabled(&runtime->motor_hal, false);
                pid_controller_reset(&runtime->balance_pid);
                stopped = true;
                ESP_LOGW(TAG, "STOP command received: motors disabled");
            } else if (command == ROBOT_CONTROL_CMD_START) {
                motor_module_set_enabled(&runtime->motor_hal, true);
                pid_controller_reset(&runtime->balance_pid);
                stopped = false;
                ESP_LOGI(TAG, "START command received: motors enabled");
            }
        }

        if (stopped) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        imu_sample_t sample;
        // Block indefinitely until new IMU data arrives from the IMU task
        if (xQueueReceive(runtime->imu_queue, &sample, portMAX_DELAY) == pdTRUE) {
            
            // 1. Get current pitch
            float current_pitch = sample.pitch_deg * PITCH_POLARITY;
            
            // 2. PID controller output in step frequency (Hz)
            float total_output = pid_controller_step(&runtime->balance_pid,
                                                     runtime->target_pitch_deg,
                                                     current_pitch,
                                                     sample.timestamp_us);
            
            // 4. Map PID output to wheel commands.
            // Left wheel follows control sign; right wheel is inverted for mirrored drivetrain orientation.
            runtime->test_cmd.left_step_hz = total_output;
            runtime->test_cmd.right_step_hz = -total_output;

            // 5. Apply commands
            motor_module_apply_command(&runtime->motor_hal, &runtime->test_cmd);
        }
    }
}

static void supervisor_task_fn(void *arg)
{
    robot_runtime_t *runtime = (robot_runtime_t *)arg;
    if (runtime == NULL) {
        vTaskDelete(NULL);
        return;
    }

    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_log_tick = last_wake;

    while (1) {
        control_setpoint_t setpoint = {0};
        (void)command_parser_poll(&runtime->command_parser, &setpoint);

        // Keep command handling responsive while preserving 1Hz telemetry logging.
        TickType_t now = xTaskGetTickCount();
        if ((now - last_log_tick) >= pdMS_TO_TICKS(1000)) {
            // Here we can eventually handle WiFi UDP, steering inputs, etc.
            // For now, just log TMC temps periodically.
            motor_module_tmc2240_test_log();
            last_log_tick = now;
        }

        // Poll command parser at 20Hz.
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(50));
    }
}

void app_main(void)
{
    maybe_start_ota_services();

    g_runtime.imu = (imu_module_t){
        .i2c_port = I2C_NUM_0,
        .sda_io = GPIO_NUM_5,
        .scl_io = GPIO_NUM_4,
        .i2c_clock_hz = 100000,
        .i2c_address = 0,
        .int_io = -1,
    };

    if (imu_module_init(&g_runtime.imu) != ESP_OK) {
        ESP_LOGW(TAG, "IMU init failed, continuing to motor test...");
    } else {
        if (imu_module_probe(&g_runtime.imu, 2000)) {
            bno08x_enable_game_rv(10000);
            bno08x_enable_gyroscope(10000);
            ESP_LOGI(TAG, "IMU Ready");
        }
    }

    if (motor_module_tmc2240_spi_init(TMC_SPI_HOST,
                                      TMC_SPI_SCLK,
                                      TMC_SPI_MOSI,
                                      TMC_SPI_MISO,
                                      TMC_LEFT_SPI_CS,
                                      1000000,
                                      false) != ESP_OK) {
        ESP_LOGE(TAG, "LEFT TMC2240 SPI init failed");
    }

    if (motor_module_tmc2240_spi_init(TMC_SPI_HOST,
                                      TMC_SPI_SCLK,
                                      TMC_SPI_MOSI,
                                      TMC_SPI_MISO,
                                      TMC_RIGHT_SPI_CS,
                                      1000000,
                                      true) != ESP_OK) {
        ESP_LOGE(TAG, "RIGHT TMC2240 SPI init failed");
    }

    TMC2240_RobotConfig_t silent_config = {
        .run_current_ma = 800,
        .hold_current_ma = 200,
        .microsteps = 16,
        .interpolate = true,
        .stealth_threshold = 500,
        .stall_sensitivity = 0,
        .cool_step_enabled = false,
    };

    motor_module_tmc2240_configure_robot_mode(&silent_config);

    g_runtime.motor_hal.left.step_pin = TMC_LEFT_STEP;
    g_runtime.motor_hal.left.dir_pin = TMC_LEFT_DIR;
    g_runtime.motor_hal.left.en_pin = TMC_EN;
    g_runtime.motor_hal.left.en_active_low = true;
    g_runtime.motor_hal.right.step_pin = TMC_RIGHT_STEP;
    g_runtime.motor_hal.right.dir_pin = TMC_RIGHT_DIR;
    g_runtime.motor_hal.right.en_pin = TMC_EN;
    g_runtime.motor_hal.right.en_active_low = true;
    g_runtime.motor_hal.max_step_hz = 2000.0f;
    motor_module_init(&g_runtime.motor_hal);
    motor_module_set_enabled(&g_runtime.motor_hal, false);

    // Initialize IMU Queue (Length 1, so we always only process the newest data)
    g_runtime.imu_queue = xQueueCreate(1, sizeof(imu_sample_t));
    // Initialize control command queue (Length 1, overwrite keeps only latest command)
    g_runtime.control_cmd_queue = xQueueCreate(1, sizeof(robot_control_command_t));
    if (g_runtime.control_cmd_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create control command queue");
        return;
    }
    robot_control_send_start();

    // Initialize PID config
    pid_controller_init(&g_runtime.balance_pid,
                        20.0f,
                        0.0f,
                        0.0f,
                        -g_runtime.motor_hal.max_step_hz,
                        g_runtime.motor_hal.max_step_hz);
    g_runtime.target_pitch_deg = 0.0f; // Stand perfectly upright

    command_parser_init(&g_runtime.command_parser);

    if (xTaskCreatePinnedToCore(imu_task_fn,
                                "imu_core0",
                                4096,
                                &g_runtime,
                                4,
                                NULL,
                                0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create IMU task on Core 0");
        return;
    }

    if (xTaskCreatePinnedToCore(control_task_fn,
                                "control_core1",
                                4096,
                                &g_runtime,
                                5, // Highest priority
                                NULL,
                                1) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create control task on Core 1");
        return;
    }

    if (xTaskCreatePinnedToCore(supervisor_task_fn,
                                "supervisor_core0",
                                4096,
                                &g_runtime,
                                2, // Low priority
                                NULL,
                                0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create supervisor task on Core 0");
        return;
    }

    vTaskDelete(NULL);
}
