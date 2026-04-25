#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/imu_module.h"
#include "hal/wifi_module.h"
#include "nvs_flash.h"
#include "supervisor/telemetry_comms.h"

static const char *TAG = "robot_main";
static TaskHandle_t s_main_task_handle = NULL;

void app_main(void)
{
    ESP_LOGI(TAG, "Robot starting (WiFi + IMU Mode)...");

    // 1. Initialize NVS (Required for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Start WiFi Access Point
    wifi_module_init_ap("Balance-Bot", "12345678");

    imu_module_t imu = {
        .sda_io = 5,
        .scl_io = 4,
        .i2c_port = I2C_NUM_0,
        .i2c_clock_hz = 50000, 
        .i2c_address = 0x4B,
    };

    s_main_task_handle = xTaskGetCurrentTaskHandle();

    // 3. Initialize IMU (I2C)
    ESP_LOGI(TAG, "Initializing BNO085 I2C (SDA=5, SCL=4)...");
    bool imu_ready = false;
    if (imu_module_init(&imu) == ESP_OK) {
        bno08x_enable_game_rv(20000); // 20ms interval
        ESP_LOGI(TAG, "IMU initialized and Game RV enabled");
        imu_ready = true;
    } else {
        ESP_LOGE(TAG, "IMU initialization failed! Check wiring (SDA=4, SCL=5), power, and pull-up resistors.");
    }

    ESP_LOGI(TAG, "IMU & Telemetry test starting...");

    imu_sample_t imu_sample;
    telemetry_comms_t telemetry;
    telemetry_comms_init(&telemetry);

    while (1) {
        int64_t now_us = esp_timer_get_time();

        // 4. Poll IMU and log orientation
        if (imu_ready) {
            imu_module_poll_and_log(&imu);
            if (imu_module_read_sample(&imu, &imu_sample) == ESP_OK) {
                // Publish to WiFi Telemetry
                robot_pose_t pose;
                pose.yaw_deg = imu_sample.yaw_deg;
                pose.pitch_deg = imu_sample.pitch_deg;
                pose.roll_deg = imu_sample.roll_deg;
                pose.tilt_deg = imu_sample.pitch_deg;
                pose.tilt_rate_dps = 0.0f;
                pose.timestamp_us = now_us;
                telemetry_comms_publish(&telemetry, 0, &pose);

                static int64_t last_pose_log_us = 0;
                if (now_us - last_pose_log_us >= 1000000) {
                    ESP_LOGI(TAG, "WiFi Pose: Y=%.2f P=%.2f R=%.2f", 
                             imu_sample.yaw_deg, imu_sample.pitch_deg, imu_sample.roll_deg);
                    last_pose_log_us = now_us;
                }
            }
        } else {
            // Blink or wait if IMU is not ready
            static int64_t last_fail_log_us = 0;
            if (now_us - last_fail_log_us >= 5000000) {
                ESP_LOGW(TAG, "IMU is not responding. Please check hardware.");
                last_fail_log_us = now_us;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
