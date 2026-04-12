#include "hal/imu_module.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "imu_module";

static void IRAM_ATTR imu_int_isr(void *arg)
{
    imu_module_t *imu = (imu_module_t *)arg;
    imu->isr_count++;

    if (imu->notify_task != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(imu->notify_task, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void imu_module_set_notify_task(imu_module_t *imu, TaskHandle_t task)
{
    imu->notify_task = task;
}

void imu_module_init(imu_module_t *imu)
{
    imu->notify_task = NULL;
    imu->isr_count = 0;

    gpio_config_t int_cfg = {
        .pin_bit_mask = (1ULL << imu->int_io),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&int_cfg));

    esp_err_t isr_service_err = gpio_install_isr_service(0);
    if (isr_service_err != ESP_OK && isr_service_err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(isr_service_err);
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add(imu->int_io, imu_int_isr, imu));

    ESP_LOGI(TAG, "IMU module initialized with INT pin %d", imu->int_io);
}

bool imu_module_read_sample(imu_module_t *imu, imu_sample_t *out_sample)
{
    (void)imu;

    out_sample->roll_deg = 0.0f;
    out_sample->pitch_deg = 0.0f - imu->tilt_zero_deg;
    out_sample->yaw_deg = 0.0f;
    out_sample->timestamp_us = esp_timer_get_time();
    return true;
}
