#include "hal/motor_module.h"

#include <math.h>

#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "motor_module";

typedef struct {
    spi_device_handle_t spi_dev;
    int cs_pin;
    bool initialized;
    const char *label; // "LEFT" or "RIGHT"
} tmc2240_dev_t;

#define TMC_LEFT  0
#define TMC_RIGHT 1
#define TMC_COUNT 2

static tmc2240_dev_t s_tmc_devs[TMC_COUNT] = {
    { .spi_dev = NULL, .cs_pin = -1, .initialized = false, .label = "LEFT" },
    { .spi_dev = NULL, .cs_pin = -1, .initialized = false, .label = "RIGHT" },
};
static int s_spi_host = -1;
static bool s_spi_bus_initialized = false;

static motor_module_t *s_active_motor = NULL;
static esp_timer_handle_t s_motor_timer_handle = NULL;

static void motor_timer_callback(void *arg)
{
    motor_module_t *motor = (motor_module_t *)arg;
    int64_t now_us = esp_timer_get_time();
    motor_module_service_step_pulses(motor, now_us);
}

static esp_err_t tmc2240_transfer_40b(int dev_idx, uint8_t tx[5], uint8_t rx[5])
{
    if (dev_idx < 0 || dev_idx >= TMC_COUNT) return ESP_ERR_INVALID_ARG;
    tmc2240_dev_t *dev = &s_tmc_devs[dev_idx];
    if (!dev->initialized || dev->spi_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Manual CS Select
    gpio_set_level(dev->cs_pin, 0);
    esp_rom_delay_us(2);

    spi_transaction_t t = {
        .length = 40,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t err = spi_device_transmit(dev->spi_dev, &t);

    // Manual CS Deselect
    esp_rom_delay_us(2);
    gpio_set_level(dev->cs_pin, 1);
    
    // TMC2240 needs time between CS de-assert and next CS assert
    esp_rom_delay_us(10);
    return err;
}

static void stepper_channel_init(stepper_channel_t *ch)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << ch->step_pin) | (1ULL << ch->dir_pin) | (1ULL << ch->en_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));

    ch->enabled = false;
    ch->target_hz = 0.0f;
    ch->last_toggle_us = 0;
    ch->step_level = false;
    gpio_set_level(ch->step_pin, 0);
    gpio_set_level(ch->dir_pin, 0);
}

static void set_enable_pin(const stepper_channel_t *ch, bool enable)
{
    int level = 0;
    if (ch->en_active_low) {
        level = enable ? 0 : 1;
    } else {
        level = enable ? 1 : 0;
    }
    gpio_set_level(ch->en_pin, level);
}

void motor_module_init(motor_module_t *motor)
{
    stepper_channel_init(&motor->left);
    stepper_channel_init(&motor->right);
    motor_module_set_enabled(motor, false);

    s_active_motor = motor;

    // Create high-resolution timer for motor pulsing (10us = 100kHz update rate)
    const esp_timer_create_args_t timer_args = {
        .callback = motor_timer_callback,
        .arg = (void *)motor,
        .name = "motor_timer",
        .dispatch_method = ESP_TIMER_TASK,
    };

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_motor_timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_motor_timer_handle, 10)); // 10us
}

void motor_module_set_enabled(motor_module_t *motor, bool enable)
{
    motor->left.enabled = enable;
    motor->right.enabled = enable;
    set_enable_pin(&motor->left, enable);
    if (motor->right.en_pin != motor->left.en_pin) {
        set_enable_pin(&motor->right, enable);
    }
}

void motor_module_apply_command(motor_module_t *motor, const motor_command_t *command)
{
    float left_hz = command->left_step_hz;
    float right_hz = command->right_step_hz;

    if (left_hz > motor->max_step_hz) {
        left_hz = motor->max_step_hz;
    }
    if (left_hz < -motor->max_step_hz) {
        left_hz = -motor->max_step_hz;
    }
    if (right_hz > motor->max_step_hz) {
        right_hz = motor->max_step_hz;
    }
    if (right_hz < -motor->max_step_hz) {
        right_hz = -motor->max_step_hz;
    }

    gpio_set_level(motor->left.dir_pin, left_hz >= 0.0f ? 1 : 0);
    if (motor->right.dir_pin != motor->left.dir_pin) {
        gpio_set_level(motor->right.dir_pin, right_hz >= 0.0f ? 1 : 0);
    }

    motor->left.target_hz = left_hz;
    motor->right.target_hz = right_hz;
}

static void service_channel(stepper_channel_t *ch, int64_t now_us)
{
    if (!ch->enabled || ch->target_hz == 0.0f) {
        ch->step_level = false;
        gpio_set_level(ch->step_pin, 0);
        return;
    }

    float abs_hz = fabsf(ch->target_hz);
    int64_t half_period_us = (int64_t)(500000.0f / abs_hz);
    if (half_period_us < 100) {
        half_period_us = 100;
    }

    if (ch->last_toggle_us == 0) {
        ch->last_toggle_us = now_us;
    }

    while ((now_us - ch->last_toggle_us) >= half_period_us) {
        ch->step_level = !ch->step_level;
        gpio_set_level(ch->step_pin, ch->step_level ? 1 : 0);
        ch->last_toggle_us += half_period_us;
    }
}

void motor_module_service_step_pulses(motor_module_t *motor, int64_t now_us)
{
    service_channel(&motor->left, now_us);
    if (motor->right.step_pin != motor->left.step_pin) {
        service_channel(&motor->right, now_us);
    }
}

esp_err_t motor_module_tmc2240_spi_init(int spi_host,
                                        int sclk_pin,
                                        int mosi_pin,
                                        int miso_pin,
                                        int cs_left,
                                        int cs_right,
                                        int spi_clock_hz)
{
    if (spi_clock_hz <= 0) {
        spi_clock_hz = 1000000;
    }

    // Initialize the SPI bus once
    if (!s_spi_bus_initialized) {
        // Configure MISO with pull-up to prevent floating noise
        gpio_config_t miso_cfg = {
            .pin_bit_mask = (1ULL << miso_pin),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&miso_cfg);

        spi_bus_config_t bus_cfg = {
            .mosi_io_num = mosi_pin,
            .miso_io_num = miso_pin,
            .sclk_io_num = sclk_pin,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 8,
        };
        esp_err_t err = spi_bus_initialize((spi_host_device_t)spi_host, &bus_cfg, SPI_DMA_DISABLED);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(err));
            return err;
        }
        s_spi_host = spi_host;
        s_spi_bus_initialized = true;
        ESP_LOGI(TAG, "SPI bus initialized (host=%d, sclk=%d, mosi=%d, miso=%d)",
                 spi_host, sclk_pin, mosi_pin, miso_pin);
    }

    // Add left device (CS=cs_left)
    if (cs_left >= 0 && !s_tmc_devs[TMC_LEFT].initialized) {
        // Configure CS with pull-up
        gpio_config_t cs_cfg = {
            .pin_bit_mask = (1ULL << cs_left),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&cs_cfg);
        gpio_set_level(cs_left, 1); // Deselect

        spi_device_interface_config_t dev_cfg = {
            .clock_speed_hz = spi_clock_hz,
            .mode = 0, // Mode 0 test
            .spics_io_num = -1, // Manual CS
            .queue_size = 2,
        };
        esp_err_t err = spi_bus_add_device((spi_host_device_t)spi_host, &dev_cfg, &s_tmc_devs[TMC_LEFT].spi_dev);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add LEFT device (CS=%d): %s", cs_left, esp_err_to_name(err));
        } else {
            s_tmc_devs[TMC_LEFT].cs_pin = cs_left;
            s_tmc_devs[TMC_LEFT].initialized = true;
            ESP_LOGI(TAG, "TMC2240 LEFT added (CS=%d)", cs_left);
        }
    }

    // Add right device (CS=cs_right)
    if (cs_right >= 0 && !s_tmc_devs[TMC_RIGHT].initialized) {
        // Configure CS with pull-up
        gpio_config_t cs_cfg = {
            .pin_bit_mask = (1ULL << cs_right),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&cs_cfg);
        gpio_set_level(cs_right, 1); // Deselect

        spi_device_interface_config_t dev_cfg = {
            .clock_speed_hz = spi_clock_hz,
            .mode = 0, // Mode 0 test
            .spics_io_num = -1, // Manual CS
            .queue_size = 2,
        };
        esp_err_t err = spi_bus_add_device((spi_host_device_t)spi_host, &dev_cfg, &s_tmc_devs[TMC_RIGHT].spi_dev);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add RIGHT device (CS=%d): %s", cs_right, esp_err_to_name(err));
        } else {
            s_tmc_devs[TMC_RIGHT].cs_pin = cs_right;
            s_tmc_devs[TMC_RIGHT].initialized = true;
            ESP_LOGI(TAG, "TMC2240 RIGHT added (CS=%d)", cs_right);
        }
    }

    return ESP_OK;
}

esp_err_t motor_module_tmc2240_write_reg(int dev_idx, uint8_t reg_addr, uint32_t value)
{
    uint8_t tx[5] = {
        (uint8_t)(reg_addr | 0x80U),
        (uint8_t)(value >> 24),
        (uint8_t)(value >> 16),
        (uint8_t)(value >> 8),
        (uint8_t)(value),
    };
    uint8_t rx[5] = {0};

    return tmc2240_transfer_40b(dev_idx, tx, rx);
}

esp_err_t motor_module_tmc2240_read_reg(int dev_idx, uint8_t reg_addr, uint32_t *value_out)
{
    if (value_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Trinamic SPI returns register data one frame later.
    uint8_t tx_cmd[5] = { (uint8_t)(reg_addr & 0x7FU), 0, 0, 0, 0 };
    uint8_t rx_cmd[5] = {0};
    esp_err_t err = tmc2240_transfer_40b(dev_idx, tx_cmd, rx_cmd);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t tx_nop[5] = {0, 0, 0, 0, 0};
    uint8_t rx_data[5] = {0};
    err = tmc2240_transfer_40b(dev_idx, tx_nop, rx_data);
    if (err != ESP_OK) {
        return err;
    }

    *value_out = ((uint32_t)rx_data[1] << 24) |
                 ((uint32_t)rx_data[2] << 16) |
                 ((uint32_t)rx_data[3] << 8) |
                 ((uint32_t)rx_data[4]);
    return ESP_OK;
}

static void tmc2240_test_log_dev(int dev_idx)
{
    tmc2240_dev_t *dev = &s_tmc_devs[dev_idx];
    if (!dev->initialized) return;

    uint32_t gconf = 0, chopconf = 0;
    float temp_c = 0.0f;
    uint8_t cs_actual = 0;

    esp_err_t gconf_err = motor_module_tmc2240_read_reg(dev_idx, 0x00, &gconf);
    esp_err_t chop_err = motor_module_tmc2240_read_reg(dev_idx, 0x6C, &chopconf);

    // Inline temp read
    uint32_t temp_data = 0;
    motor_module_tmc2240_read_reg(dev_idx, 0x51, &temp_data);
    temp_c = (float)((uint16_t)(temp_data & 0x1FFF) - 2038) / 7.7f;

    // Inline CS_ACTUAL read
    uint32_t drv_status = 0;
    motor_module_tmc2240_read_reg(dev_idx, 0x6F, &drv_status);
    cs_actual = (uint8_t)((drv_status >> 16) & 0x1F);

    if (gconf_err == ESP_OK && chop_err == ESP_OK) {
        float est_ma = ((float)cs_actual + 1.0f) / 32.0f * 3000.0f * 0.707f;
        ESP_LOGI(TAG, "[%s CS=%d] GCONF=0x%08lX CHOP=0x%08lX TEMP=%.1fC CS=%d (~%.0fmA)",
                 dev->label, dev->cs_pin,
                 (unsigned long)gconf,
                 (unsigned long)chopconf,
                 temp_c,
                 (int)cs_actual,
                 est_ma);
    } else {
        ESP_LOGW(TAG, "[%s CS=%d] SPI read failed", dev->label, dev->cs_pin);
    }
}

void motor_module_tmc2240_test_log(void)
{
    tmc2240_test_log_dev(TMC_LEFT);
    tmc2240_test_log_dev(TMC_RIGHT);
}

static void tmc2240_configure_dev(int dev_idx)
{
    tmc2240_dev_t *dev = &s_tmc_devs[dev_idx];
    if (!dev->initialized) return;

    // CHOPCONF: proven values from working example, TOFF=5, MRES=4 (16 microsteps)
    uint32_t chopconf = 0x10410150 | (0x04 << 24) | 0x05;
    motor_module_tmc2240_write_reg(dev_idx, 0x6C, chopconf);

    // IHOLD_IRUN: IHOLD=6, IRUN=14, IHOLDDELAY=6
    uint32_t ihold_irun = (6 << 16) | (14 << 8) | (6);
    motor_module_tmc2240_write_reg(dev_idx, 0x10, ihold_irun);

    // GCONF: defaults
    motor_module_tmc2240_write_reg(dev_idx, 0x00, 0x00000000);

    // Verify
    uint32_t readback = 0;
    motor_module_tmc2240_read_reg(dev_idx, 0x6C, &readback);
    if ((readback & 0x0F) != 0x05) {
        ESP_LOGE(TAG, "[%s CS=%d] CHOPCONF verify FAILED (read=0x%08lX, TOFF=%ld)",
                 dev->label, dev->cs_pin,
                 (unsigned long)readback, (unsigned long)(readback & 0x0F));
    } else {
        ESP_LOGI(TAG, "[%s CS=%d] Configured OK (CHOPCONF=0x%08lX)",
                 dev->label, dev->cs_pin, (unsigned long)readback);
    }
}

void motor_module_tmc2240_configure_robot_mode(void)
{
    tmc2240_configure_dev(TMC_LEFT);
    tmc2240_configure_dev(TMC_RIGHT);
}
// motor_module_tmc2240_full_quick_setup was removed. 
// Use motor_module_init and motor_module_tmc2240_spi_init with explicit pins instead.

esp_err_t motor_module_tmc2240_get_temp(int dev_idx, float *temp_c)
{
    if (temp_c == NULL) return ESP_ERR_INVALID_ARG;

    uint32_t data = 0;
    esp_err_t err = motor_module_tmc2240_read_reg(dev_idx, 0x51, &data);
    if (err != ESP_OK) return err;

    // TMC2240 Temperature formula from sample code: (ADC_DATA - 2038) / 7.7
    *temp_c = (float)((uint16_t)(data & 0x1FFF) - 2038) / 7.7f;
    return ESP_OK;
}

esp_err_t motor_module_tmc2240_get_current_scaling(int dev_idx, uint8_t *cs_actual)
{
    if (cs_actual == NULL) return ESP_ERR_INVALID_ARG;

    uint32_t data = 0;
    esp_err_t err = motor_module_tmc2240_read_reg(dev_idx, 0x6F, &data); // DRV_STATUS
    if (err != ESP_OK) return err;

    // CS_ACTUAL is in bits [20:16]
    *cs_actual = (uint8_t)((data >> 16) & 0x1F);
    return ESP_OK;
}

// NOTE: motor_module_tmc2240_spi_velocity_test() was removed.
// The TMC2240 does NOT have an internal motion controller (unlike TMC5130/5160).
// Registers 0x20-0x2B (RAMPMODE, VMAX, AMAX, etc.) do not exist on the TMC2240.
// Use motor_module_apply_command() with STEP/DIR pulses instead.
