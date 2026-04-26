#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/imu_module.h"
#include "hal/motor_module.h"
#include "hal/wifi_module.h"
#include "nvs_flash.h"
#include "supervisor/telemetry_comms.h"

static const char *TAG = "robot_main";

void app_main(void)
{
    ESP_LOGI(TAG, "Motor Test Starting (WiFi + IMU + Motor)...");

    // 1. Initialize NVS (Required for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Start WiFi Access Point
    wifi_module_init_ap("Balance-Bot", "12345678");

    // 3. Initialize IMU (I2C) - SDA=5, SCL=4
    imu_module_t imu = {
        .sda_io = 5,
        .scl_io = 4,
        .i2c_port = I2C_NUM_0,
        .i2c_clock_hz = 50000, 
        .i2c_address = 0x4B,
    };
    (void)imu_module_init(&imu);
    (void)bno08x_enable_game_rv(20000);

    // 4. Safety: Motor disabled to prevent back-powering
    ESP_LOGI(TAG, "Motor safety-off mode enabled. WiFi + IMU only.");

    imu_sample_t imu_sample;
    telemetry_comms_t telemetry;
    telemetry_comms_init(&telemetry);

    while (1) {
        int64_t now_us = esp_timer_get_time();

        // IMU & Telemetry (Keep running in background)
        imu_module_poll_and_log(&imu);
        if (imu_module_read_sample(&imu, &imu_sample) == ESP_OK) {
            robot_pose_t pose;
            pose.yaw_deg = imu_sample.yaw_deg;
            pose.pitch_deg = imu_sample.pitch_deg;
            pose.roll_deg = imu_sample.roll_deg;
            pose.tilt_deg = imu_sample.pitch_deg;
            pose.timestamp_us = now_us;
            telemetry_comms_publish(&telemetry, 0, &pose);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
