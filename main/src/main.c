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

static const char *TAG = "i2c_mode_check";
static TaskHandle_t s_main_task_handle = NULL;

// TMC2240 SPI test wiring (adjust to your board).
#define TMC_SPI_HOST SPI2_HOST
#define TMC_SPI_SCLK GPIO_NUM_16
#define TMC_SPI_MOSI GPIO_NUM_15
#define TMC_SPI_MISO GPIO_NUM_10
#define TMC_SPI_CS   GPIO_NUM_9
#define TMC_RIGHT_SPI_CS GPIO_NUM_39
#define TMC_DIAG0    GPIO_NUM_6
#define TMC_RIGHT_DIAG0 GPIO_NUM_41
#define TMC_EN       GPIO_NUM_18
#define TMC_STEP     GPIO_NUM_8
#define TMC_DIR      GPIO_NUM_11

void app_main(void)
{
    imu_module_t imu = {
        .i2c_port = I2C_NUM_0,
        .sda_io = GPIO_NUM_5,
        .scl_io = GPIO_NUM_4,
        .i2c_clock_hz = 100000,
        .i2c_address = 0,
        .int_io = -1, // Set to your HINT pin if used
    };

    s_main_task_handle = xTaskGetCurrentTaskHandle();

    // 1. Initialize IMU
    if (imu_module_init(&imu) != ESP_OK) {
        ESP_LOGW(TAG, "IMU init failed, continuing to motor test...");
    } else {
        if (imu_module_probe(&imu, 2000)) {
             imu_module_enable_default_reports(&imu);
             ESP_LOGI(TAG, "IMU Ready");
        }
    }

    // 2. Initialize TMC2240 SPI (Left-only Isolation Test)
    // Args: host, sclk, mosi, miso, cs, clock
    if (motor_module_tmc2240_spi_init(TMC_SPI_HOST,
                                      TMC_SPI_SCLK,
                                      TMC_SPI_MOSI,
                                      TMC_SPI_MISO,
                                      TMC_SPI_CS,
                                      100000) != ESP_OK) {
        ESP_LOGE(TAG, "TMC2240 SPI init failed");
    }

    if (motor_module_tmc2240_right_spi_init(TMC_SPI_HOST,
                                            TMC_SPI_SCLK,
                                            TMC_SPI_MOSI,
                                            TMC_SPI_MISO,
                                            TMC_RIGHT_SPI_CS,
                                            100000) != ESP_OK) {
        ESP_LOGE(TAG, "RIGHT TMC2240 SPI init failed");
    }

    // 3. Initialize Motor HAL and start 500Hz pulse test
    static motor_module_t motor_hal = {0};
    motor_hal.left.step_pin = TMC_STEP;
    motor_hal.left.dir_pin = TMC_DIR;
    motor_hal.left.en_pin = TMC_EN;
    motor_hal.left.en_active_low = true;
    motor_hal.max_step_hz = 5000.0f;
    
    motor_module_init(&motor_hal);
    motor_module_set_enabled(&motor_hal, true);
    
    motor_command_t cmd = { .left_step_hz = 500.0f, .right_step_hz = 0.0f };
    motor_module_apply_command(&motor_hal, &cmd);

    ESP_LOGI(TAG, "Starting 500Hz Pulse Test on LEFT motor...");

    int64_t last_tmc_log_us = 0;
    while (1) {
        imu_module_poll_and_log(&imu);

        int64_t now_us = esp_timer_get_time();
        if ((now_us - last_tmc_log_us) >= 1000000) {
            motor_module_tmc2240_test_log();
            motor_module_tmc2240_right_test_log();
            last_tmc_log_us = now_us;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
