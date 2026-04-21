#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/imu_module.h"

static const char *TAG = "i2c_mode_check";
static TaskHandle_t s_main_task_handle = NULL;

static void imu_hint_isr_handler(void *arg)
{
    TaskHandle_t task_handle = (TaskHandle_t)arg;
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(task_handle, &higher_priority_task_woken);
    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void app_main(void)
{
    imu_module_t imu = {
        .i2c_port = I2C_NUM_0,
        .sda_io = GPIO_NUM_4,
        .scl_io = GPIO_NUM_5,
        .i2c_clock_hz = 100000,
        .i2c_address = 0,
    };

    s_main_task_handle = xTaskGetCurrentTaskHandle();

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

    gpio_config_t hint_cfg = {
        .pin_bit_mask = 1ULL << imu.int_io,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&hint_cfg));
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(gpio_isr_handler_add(imu.int_io, imu_hint_isr_handler, s_main_task_handle));

    if (bno08x_enable_accelerometer(5000) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable Accelerometer report");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    ESP_LOGI(TAG, "BNO085 communication detected through SH-2.");

    while (1) {
        // Fall back to periodic servicing in case HINT edges are missed.
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
        imu_module_poll_and_log(&imu);
    }
}
