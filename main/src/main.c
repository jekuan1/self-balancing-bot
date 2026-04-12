#include <inttypes.h>
#include <stdio.h>

#include "esp_chip_info.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "phase1_test";

void app_main(void)
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    ESP_LOGI(TAG, "Phase 1: Brains-only bring-up test");
    ESP_LOGI(TAG, "CPU cores: %d", chip_info.cores);

    size_t psram_size = esp_psram_get_size();
    ESP_LOGI(TAG, "PSRAM size (bytes): %u", (unsigned)psram_size);

    if (psram_size == 8388608U) {
        ESP_LOGI(TAG, "PSRAM check PASSED (8MB detected)");
    } else if (psram_size == 0U) {
        ESP_LOGW(TAG, "PSRAM check FAILED (0 bytes). Verify module config and wiring.");
    } else {
        ESP_LOGW(TAG, "PSRAM check unexpected value: %u bytes", (unsigned)psram_size);
    }

    uint32_t tick = 0;
    while (1) {
        ESP_LOGI(TAG, "Hello World heartbeat: %" PRIu32, tick++);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
