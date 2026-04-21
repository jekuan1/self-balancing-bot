#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/imu_module.h"

static const char *TAG = "i2c_mode_check";
static const uint32_t IMU_SERVICE_PERIOD_MS = 1;

void app_main(void)
{
    imu_module_t imu = {
        .i2c_port = I2C_NUM_0,
        .sda_io = GPIO_NUM_4,
        .scl_io = GPIO_NUM_5,
        .i2c_clock_hz = 100000,
        .i2c_address = 0,
    };

    if (imu_module_init(&imu) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BNO085");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    if (!imu_module_probe(&imu, 4000)) {
        ESP_LOGE(TAG, "No SH-2 response from BNO085");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    ESP_LOGI(TAG, "BNO085 communication detected through SH-2.");
    imu_module_enable_default_reports(&imu);

    while (1) {
        imu_module_poll_and_log(&imu);
        vTaskDelay(pdMS_TO_TICKS(IMU_SERVICE_PERIOD_MS));
    }
}
