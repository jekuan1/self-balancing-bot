#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "control/pid_controller.h"
#include "hal/imu_module.h"
#include "hal/motor_module.h"
#include "hal/ota_module.h"
#include "hal/safety_watchdog.h"
#include "hal/wifi_module.h"
#include "robot_control.h"
#include "robot_types.h"
#include "control/state_estimator.h"
#include "supervisor/command_parser.h"
#include "supervisor/udp_command_receiver.h"
#include "supervisor/udp_logging.h"
#include "supervisor/telemetry_comms.h"

static const char *TAG = "motor_test";

#define ENABLE_OTA_AP 1
#define OTA_AP_SSID "BalanceBot"
#define OTA_AP_PASSWORD "balance123"

typedef struct {
    imu_module_t imu;
    motor_module_t motor_hal;
    safety_watchdog_t safety_watchdog;
    motor_command_t test_cmd;
    QueueHandle_t imu_queue;
    QueueHandle_t control_cmd_queue;
    command_parser_t command_parser;
    pid_controller_t balance_pid;
    state_estimator_t state_estimator;
    telemetry_comms_t telemetry_comms;
    robot_pose_t current_pose;
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

bool robot_control_tune_pid(float kp, float ki, float kd)
{
    g_runtime.balance_pid.kp = kp;
    g_runtime.balance_pid.ki = ki;
    g_runtime.balance_pid.kd = kd;
    pid_controller_reset(&g_runtime.balance_pid);
    ESP_LOGI(TAG, "PID tuned: kp=%.3f ki=%.3f kd=%.3f", kp, ki, kd);
    return true;
}

bool robot_control_set_target(float pitch_deg)
{
    g_runtime.target_pitch_deg = pitch_deg;
    pid_controller_reset(&g_runtime.balance_pid);
    ESP_LOGI(TAG, "Target pitch set: %.3f deg", pitch_deg);
    return true;
}

float robot_control_get_pitch(void)
{
    return g_runtime.current_pose.pitch_deg;
}

static void maybe_start_ota_services(void)
{
#if ENABLE_OTA_AP
    // Initialize NVS first (required by WiFi)
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated/updated, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    esp_err_t wifi_err = wifi_module_init_ap(OTA_AP_SSID, OTA_AP_PASSWORD);
    if (wifi_err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi AP init failed: %s", esp_err_to_name(wifi_err));
        return;
    }

    esp_err_t ota_err = ota_module_init();
    if (ota_err != ESP_OK) {
        ESP_LOGE(TAG, "OTA server init failed: %s", esp_err_to_name(ota_err));
        return;
    }
    ESP_LOGI(TAG, "OTA AP mode enabled. SSID=%s, open http://192.168.4.1/", OTA_AP_SSID);
    
    // Start UDP telemetry; command receiver is started later after control state is ready.
    udp_logging_init("192.168.4.255", 5556);
    ESP_LOGI(TAG, "UDP telemetry enabled on port 5556");
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
                state_estimator_update(&runtime->state_estimator, &sample, &runtime->current_pose);
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

    motor_module_set_enabled(&runtime->motor_hal, false);
    ESP_LOGI(TAG, "Control Task Started in STOP mode. Waiting for START command...");

    // Pitch polarity can be flipped if IMU mounting orientation is inverted.
    const float PITCH_POLARITY = -1.0f;
    bool stopped = true;

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
            float current_pitch_rate = runtime->current_pose.tilt_rate_dps * PITCH_POLARITY;
            
            // 2. PID controller output in step frequency (Hz)
            float total_output = pid_controller_step(&runtime->balance_pid,
                                                     runtime->target_pitch_deg,
                                                     current_pitch,
                                                     current_pitch_rate,
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
    bool watchdog_latched = false;

    while (1) {
        if (safety_watchdog_is_tripped(&runtime->safety_watchdog)) {
            if (!watchdog_latched) {
                watchdog_latched = true;
                (void)robot_control_send_stop();
                ESP_LOGE(TAG,
                         "Safety watchdog tripped (count=%lu): issuing STOP",
                         (unsigned long)runtime->safety_watchdog.trip_count);
            }
        }

        control_setpoint_t setpoint = {0};
        (void)command_parser_poll(&runtime->command_parser, &setpoint);

        // Keep command handling responsive while preserving high-rate telemetry.
        TickType_t now = xTaskGetTickCount();
        
        // Publish telemetry at 50Hz (via telemetry_comms internal rate limiting)
        telemetry_comms_publish(&runtime->telemetry_comms, ROBOT_STATE_READY, &runtime->current_pose);
        
        // Log TMC temps at 1Hz
        if ((now - last_log_tick) >= pdMS_TO_TICKS(1000)) {
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

    // BNO085 needs time to boot after power-on; without this delay the probe
    // fails on fast resets before the sensor's SH-2 firmware is ready.
    vTaskDelay(pdMS_TO_TICKS(150));

    bool imu_ready = false;
    for (int attempt = 1; attempt <= 3 && !imu_ready; attempt++) {
        if (imu_module_init(&g_runtime.imu) != ESP_OK) {
            ESP_LOGW(TAG, "IMU init failed (attempt %d/3)", attempt);
        } else if (imu_module_probe(&g_runtime.imu, 2000)) {
            bno08x_enable_game_rv(10000);
            bno08x_enable_gyroscope(10000);
            ESP_LOGI(TAG, "IMU Ready (attempt %d/3)", attempt);
            imu_ready = true;
        } else {
            ESP_LOGW(TAG, "IMU probe timed out (attempt %d/3)", attempt);
        }
        if (!imu_ready && attempt < 3) {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
    if (!imu_ready) {
        ESP_LOGW(TAG, "IMU not detected after 3 attempts, continuing without IMU");
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
        .run_current_ma = 500,
        .hold_current_ma = 100,
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
    g_runtime.motor_hal.max_step_hz = 3000.0f;
    motor_module_init(&g_runtime.motor_hal);
    motor_module_set_enabled(&g_runtime.motor_hal, false);

    // Motor test: left 3s, right 3s, both 3s
    ESP_LOGI(TAG, "[MOTOR TEST] LEFT 3s...");
    {
        motor_command_t cmd = { .left_step_hz = 1000.0f, .right_step_hz = 0.0f };
        motor_module_set_enabled(&g_runtime.motor_hal, true);
        motor_module_apply_command(&g_runtime.motor_hal, &cmd);
        vTaskDelay(pdMS_TO_TICKS(3000));
        motor_module_set_enabled(&g_runtime.motor_hal, false);
    }
    ESP_LOGI(TAG, "[MOTOR TEST] RIGHT 3s...");
    {
        motor_command_t cmd = { .left_step_hz = 0.0f, .right_step_hz = 1000.0f };
        motor_module_set_enabled(&g_runtime.motor_hal, true);
        motor_module_apply_command(&g_runtime.motor_hal, &cmd);
        vTaskDelay(pdMS_TO_TICKS(3000));
        motor_module_set_enabled(&g_runtime.motor_hal, false);
    }
    ESP_LOGI(TAG, "[MOTOR TEST] BOTH 3s...");
    {
        motor_command_t cmd = { .left_step_hz = 1000.0f, .right_step_hz = 1000.0f };
        motor_module_set_enabled(&g_runtime.motor_hal, true);
        motor_module_apply_command(&g_runtime.motor_hal, &cmd);
        vTaskDelay(pdMS_TO_TICKS(3000));
        motor_module_set_enabled(&g_runtime.motor_hal, false);
    }
    ESP_LOGI(TAG, "[MOTOR TEST] Done");

    // Disable watchdog interrupts to reduce motor control jitter
    // (DIAG interrupts were causing audible artifacts in motor noise)
    g_runtime.safety_watchdog.diag_left_io = -1;
    g_runtime.safety_watchdog.diag_right_io = -1;
    safety_watchdog_init(&g_runtime.safety_watchdog);

    // Initialize IMU Queue (Length 1, so we always only process the newest data)
    g_runtime.imu_queue = xQueueCreate(1, sizeof(imu_sample_t));
    // Initialize control command queue (Length 1, overwrite keeps only latest command)
    g_runtime.control_cmd_queue = xQueueCreate(1, sizeof(robot_control_command_t));
    if (g_runtime.control_cmd_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create control command queue");
        return;
    }
    // Initialize state estimator for pose calculation
    state_estimator_init(&g_runtime.state_estimator);

    // Initialize telemetry on port 1234
    telemetry_comms_init(&g_runtime.telemetry_comms);

    // Initialize PID config
    pid_controller_init(&g_runtime.balance_pid,
                        400.0f,
                        0.0f,
                        25.0f,
                        -g_runtime.motor_hal.max_step_hz,
                        g_runtime.motor_hal.max_step_hz);
    g_runtime.target_pitch_deg = 0.0f; // Stand perfectly upright

    command_parser_init(&g_runtime.command_parser);

    if (udp_command_receiver_init() != ESP_OK) {
        ESP_LOGE(TAG, "UDP command receiver failed to start");
    } else {
        ESP_LOGI(TAG, "UDP command receiver enabled on port 5555");
    }

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
