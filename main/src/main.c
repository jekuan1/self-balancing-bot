#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/imu_module.h"

static const char *TAG = "spi_mode_check";
static const uint32_t IMU_SERVICE_PERIOD_MS = 1;

void app_main(void)
{
    imu_module_t imu = {
        .spi_host = SPI2_HOST,
        .mosi_io = GPIO_NUM_8,  // Safe
        .miso_io = GPIO_NUM_18, // Safe
        .sclk_io = GPIO_NUM_7,  // Safe
        .cs_io = GPIO_NUM_2,    // Definitely safe
        .int_io = GPIO_NUM_6,   // Safe
        .rst_io = GPIO_NUM_42,
        .wake_io = GPIO_NUM_41,
        .spi_clock_hz = 1000000,
        .spi_mode = 3,
        .int_active_low = true,
    };

    bool probe_ok = false;
    const int max_attempts = 1;
    
    for (int attempt = 1; attempt <= max_attempts; attempt++) {
        ESP_LOGI(TAG, "BNO085 attempt %d/%d using SH-2 library path", attempt, max_attempts);

        // 1. Initialize hardware and open the SH-2 stack.
        if (imu_module_init(&imu) != ESP_OK) {
            ESP_LOGE(TAG, "Init failed (bus or SH-2 open error). Retrying...");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        // 2. Probe with a library call instead of manual packet parsing.
        if (imu_module_probe(&imu, 4000)) {
            probe_ok = true;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(150));
    }
    
    if (!probe_ok) {
        ESP_LOGE(TAG, "No SH-2 response from BNO085 after %d retries.", max_attempts);
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
