#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/imu_module.h"
#include "hal/motor_module.h"

static const char *TAG = "motor_test";

typedef struct {
    imu_module_t imu;
    motor_module_t motor_hal;
    motor_command_t test_cmd;
    int64_t motion_start_us;
    TaskHandle_t control_task;
    TaskHandle_t supervisor_task;
} robot_runtime_t;

static robot_runtime_t g_runtime = {0};

// TMC2240 SPI test wiring (adjust to your board).
#define TMC_SPI_HOST        SPI2_HOST
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

static void control_task_fn(void *arg)
{
    robot_runtime_t *runtime = (robot_runtime_t *)arg;
    if (runtime == NULL) {
        vTaskDelete(NULL);
        return;
    }

    int64_t start_time = esp_timer_get_time();
    runtime->motion_start_us = start_time;

    motor_module_set_enabled(&runtime->motor_hal, true);
    ESP_LOGI(TAG, "AT#1: holding standstill for StealthChop2 tuning before first move...");
    vTaskDelay(pdMS_TO_TICKS(150));

    runtime->motion_start_us = esp_timer_get_time();
    
    // Phase 1: Left motor only for 3 seconds
    ESP_LOGI(TAG, "Phase 1: Running LEFT motor for 3 seconds...");
    runtime->test_cmd.left_step_hz = 1000.0f;
    runtime->test_cmd.right_step_hz = 0.0f;
    motor_module_apply_command(&runtime->motor_hal, &runtime->test_cmd);

    TickType_t last_wake = xTaskGetTickCount();
    int phase = 1;
    
    while (1) {
        int64_t now_us = esp_timer_get_time();
        int64_t elapsed_us = now_us - runtime->motion_start_us;

        if (phase == 1 && elapsed_us >= 3000000) {
            // Switch to Phase 2: Right motor only for 3 seconds
            ESP_LOGI(TAG, "Phase 1 complete. Phase 2: Running RIGHT motor for 3 seconds...");
            runtime->test_cmd.left_step_hz = 0.0f;
            runtime->test_cmd.right_step_hz = 1000.0f;
            motor_module_apply_command(&runtime->motor_hal, &runtime->test_cmd);
            phase = 2;
            runtime->motion_start_us = now_us;
        }

        if (phase == 2 && (now_us - runtime->motion_start_us) >= 3000000) {
            // Stop both motors
            ESP_LOGI(TAG, "Phase 2 complete. Stopping both motors.");
            runtime->test_cmd.left_step_hz = 0.0f;
            runtime->test_cmd.right_step_hz = 0.0f;
            motor_module_apply_command(&runtime->motor_hal, &runtime->test_cmd);
            phase = 3;
            ESP_LOGI(TAG, "Test Complete: Right 3s + Left 3s done. Motors stopped.");
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}

static void supervisor_task_fn(void *arg)
{
    robot_runtime_t *runtime = (robot_runtime_t *)arg;
    if (runtime == NULL) {
        vTaskDelete(NULL);
        return;
    }

    int64_t last_print_us = 0;
    int64_t last_wait_us = 0;

    while (1) {
        if (runtime->imu.i2c_dev != NULL) {
            imu_module_poll_and_log(&runtime->imu);
        }

        imu_sample_t sample;
        if (imu_module_read_sample(&runtime->imu, &sample) == ESP_OK) {
            int64_t now_us = esp_timer_get_time();
            if (now_us - last_print_us >= 1000000) {
                // ESP_LOGI(TAG, "Yaw: %8.3f | Pitch: %8.3f | Roll: %8.3f",
                //          (double)sample.yaw_deg,
                //          (double)sample.pitch_deg,
                //          (double)sample.roll_deg);
                motor_module_tmc2240_test_log();
                motor_module_tmc2240_debug_left();
                motor_module_tmc2240_debug_right();
                last_print_us = now_us;
            }
        } else {
            int64_t now_us = esp_timer_get_time();
            if (now_us - last_wait_us >= 1000000) {
                ESP_LOGI(TAG, "Waiting for valid IMU orientation... (is the sensor connected?)");
                last_wait_us = now_us;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
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

    if (xTaskCreatePinnedToCore(control_task_fn,
                                "control_core1",
                                4096,
                                &g_runtime,
                                5,
                                &g_runtime.control_task,
                                1) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create control task on Core 1");
        return;
    }

    if (xTaskCreatePinnedToCore(supervisor_task_fn,
                                "supervisor_core0",
                                4096,
                                &g_runtime,
                                2,
                                &g_runtime.supervisor_task,
                                0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create supervisor task on Core 0");
        return;
    }

    vTaskDelete(NULL);
}
