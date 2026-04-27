#include "hal/safety_watchdog.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "safety_watchdog";

static void IRAM_ATTR diag_isr(void *arg)
{
    safety_watchdog_t *watchdog = (safety_watchdog_t *)arg;
    watchdog->tripped = true;
    watchdog->trip_count++;
}

void safety_watchdog_init(safety_watchdog_t *watchdog)
{
    watchdog->tripped = false;
    watchdog->trip_count = 0;

    uint64_t pin_mask = 0;
    if (watchdog->diag_left_io >= 0) pin_mask |= (1ULL << watchdog->diag_left_io);
    if (watchdog->diag_right_io >= 0) pin_mask |= (1ULL << watchdog->diag_right_io);

    if (pin_mask > 0) {
        gpio_config_t cfg = {
            .pin_bit_mask = pin_mask,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_NEGEDGE,
        };
        ESP_ERROR_CHECK(gpio_config(&cfg));

        esp_err_t isr_service_err = gpio_install_isr_service(0);
        if (isr_service_err != ESP_OK && isr_service_err != ESP_ERR_INVALID_STATE) {
            ESP_ERROR_CHECK(isr_service_err);
        }

        if (watchdog->diag_left_io >= 0) {
            ESP_ERROR_CHECK(gpio_isr_handler_add(watchdog->diag_left_io, diag_isr, watchdog));
        }
        if (watchdog->diag_right_io >= 0) {
            ESP_ERROR_CHECK(gpio_isr_handler_add(watchdog->diag_right_io, diag_isr, watchdog));
        }

        ESP_LOGI(TAG, "Safety watchdog initialized (L=%d, R=%d)", watchdog->diag_left_io, watchdog->diag_right_io);
    } else {
        ESP_LOGW(TAG, "Watchdog initialized with NO pins (diag disabled)");
    }
}

bool safety_watchdog_is_tripped(const safety_watchdog_t *watchdog)
{
    return watchdog->tripped;
}

void safety_watchdog_clear(safety_watchdog_t *watchdog)
{
    watchdog->tripped = false;
}
