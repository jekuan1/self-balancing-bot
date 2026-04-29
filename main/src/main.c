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
    int64_t last_dir_toggle_us;
    bool motion_stopped;
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
#define TMC_LEFT_SPI_CS     GPIO_NUM_12
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

    runtime->motion_start_us = esp_timer_get_time();
    runtime->last_dir_toggle_us = 0;
    runtime->motion_stopped = false;

    motor_module_set_enabled(&runtime->motor_hal, true);
    motor_module_apply_command(&runtime->motor_hal, &runtime->test_cmd);

    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        int64_t now_us = esp_timer_get_time();

        if (!runtime->motion_stopped && (now_us - runtime->motion_start_us) >= 10000000) {
            runtime->test_cmd.left_step_hz = 0.0f;
            runtime->test_cmd.right_step_hz = 0.0f;
            motor_module_apply_command(&runtime->motor_hal, &runtime->test_cmd);
            motor_module_set_enabled(&runtime->motor_hal, false);
            runtime->motion_stopped = true;
        }

        if (!runtime->motion_stopped && (now_us - runtime->last_dir_toggle_us) >= 3000000) {
            runtime->test_cmd.left_step_hz = -runtime->test_cmd.left_step_hz;
            runtime->test_cmd.right_step_hz = -runtime->test_cmd.right_step_hz;
            motor_module_apply_command(&runtime->motor_hal, &runtime->test_cmd);
            runtime->last_dir_toggle_us = now_us;
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

    int64_t last_tmc_log_us = 0;

    while (1) {
        imu_module_poll_and_log(&runtime->imu);

        int64_t now_us = esp_timer_get_time();
        if ((now_us - last_tmc_log_us) >= 1000000) {
            motor_module_tmc2240_test_log();
            last_tmc_log_us = now_us;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void)
{
    imu_module_t imu = {
        .i2c_port = I2C_NUM_0,
        .sda_io = GPIO_NUM_5,
        .scl_io = GPIO_NUM_4,
        .i2c_clock_hz = 100000,
        .i2c_address = 0,
        .int_io = -1,
    };

    if (imu_module_init(&imu) != ESP_OK) {
        ESP_LOGW(TAG, "IMU init failed, continuing to motor test...");
    } else {
        if (imu_module_probe(&imu, 2000)) {
            imu_module_enable_default_reports(&imu);
            ESP_LOGI(TAG, "IMU Ready");
        }
    }

    if (motor_module_tmc2240_spi_init(TMC_SPI_HOST,
                                      TMC_SPI_SCLK,
                                      TMC_SPI_MOSI,
                                      TMC_SPI_MISO,
                                      TMC_LEFT_SPI_CS,
                                      1000000) != ESP_OK) {
        ESP_LOGE(TAG, "LEFT TMC2240 SPI init failed");
    }

    if (motor_module_tmc2240_right_spi_init(TMC_SPI_HOST,
                                            TMC_SPI_SCLK,
                                            TMC_SPI_MOSI,
                                            TMC_SPI_MISO,
                                            TMC_RIGHT_SPI_CS,
                                            1000000) != ESP_OK) {
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

    g_runtime.imu = imu;
    g_runtime.motor_hal.left.step_pin = TMC_LEFT_STEP;
    g_runtime.motor_hal.left.dir_pin = TMC_LEFT_DIR;
    g_runtime.motor_hal.left.en_pin = TMC_EN;
    g_runtime.motor_hal.left.en_active_low = true;
    g_runtime.motor_hal.right.step_pin = TMC_RIGHT_STEP;
    g_runtime.motor_hal.right.dir_pin = TMC_RIGHT_DIR;
    g_runtime.motor_hal.right.en_pin = TMC_EN;
    g_runtime.motor_hal.right.en_active_low = true;
    g_runtime.motor_hal.max_step_hz = 2000.0f;
    g_runtime.test_cmd = (motor_command_t) {
        .left_step_hz = 300.0f,
        .right_step_hz = 300.0f,
    };

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
