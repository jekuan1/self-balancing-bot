#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/wifi_module.h"
#include "hal/ota_module.h"
#include "hal/motor_module.h"
#include "hal/imu_module.h"
#include "supervisor/udp_logging.h"
#include "robot_system.h"

static const char *TAG = "robot_main";

void app_main(void)
{
    ESP_LOGI(TAG, "Robot starting...");

    // 1. Initialize NVS (Required for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Start WiFi Access Point
    wifi_module_init_ap("Balance-Bot", "12345678");

    // 3. Start Wireless Logging (Broadcasts Log to everything on WiFi)
    udp_logging_init("255.255.255.255", 1234);

    // 4. Start OTA Server for Wireless Flashing
    ota_module_init();

    // 5. Start Robot System (Initializes IMU, Motors, and Control Task)
    robot_system_start();

    // Main loop stays empty, work is done in background tasks
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
