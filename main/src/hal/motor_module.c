#include "hal/motor_module.h"

#include <math.h>

#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "motor_module";

typedef struct {
    spi_device_handle_t spi_dev;
    int spi_host;
    bool initialized;
    const char *label;
} tmc2240_spi_ctx_t;

static tmc2240_spi_ctx_t s_tmc2240_left_ctx = {
    .spi_dev = NULL,
    .spi_host = -1,
    .initialized = false,
    .label = "LEFT",
};

static tmc2240_spi_ctx_t s_tmc2240_right_ctx = {
    .spi_dev = NULL,
    .spi_host = -1,
    .initialized = false,
    .label = "RIGHT",
};

static bool s_tmc2240_bus_initialized = false;
static int s_tmc2240_bus_host = -1;

static esp_err_t tmc2240_transfer_40b_ctx(tmc2240_spi_ctx_t *ctx, uint8_t tx[5], uint8_t rx[5])
{
    if (ctx == NULL || !ctx->initialized || ctx->spi_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    spi_transaction_t t = {
        .length = 40,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    return spi_device_transmit(ctx->spi_dev, &t);
}

static esp_err_t tmc2240_read_reg_raw_ctx(tmc2240_spi_ctx_t *ctx,
                                          uint8_t reg_addr,
                                          uint8_t tx_cmd[5],
                                          uint8_t rx_cmd[5],
                                          uint8_t rx_data[5])
{
    if (ctx == NULL || tx_cmd == NULL || rx_cmd == NULL || rx_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    tx_cmd[0] = (uint8_t)(reg_addr & 0x7FU);
    tx_cmd[1] = 0;
    tx_cmd[2] = 0;
    tx_cmd[3] = 0;
    tx_cmd[4] = 0;

    esp_err_t err = tmc2240_transfer_40b_ctx(ctx, tx_cmd, rx_cmd);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t tx_nop[5] = {0, 0, 0, 0, 0};
    return tmc2240_transfer_40b_ctx(ctx, tx_nop, rx_data);
}

static esp_err_t tmc2240_spi_init_ctx(tmc2240_spi_ctx_t *ctx,
                                      int spi_host,
                                      int sclk_pin,
                                      int mosi_pin,
                                      int miso_pin,
                                      int cs_pin,
                                      int spi_clock_hz)
{
    if (ctx == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (ctx->initialized) {
        return ESP_OK;
    }

    if (spi_clock_hz <= 0) {
        spi_clock_hz = 1000000;
    }

    if (!s_tmc2240_bus_initialized) {
        spi_bus_config_t bus_cfg = {
            .mosi_io_num = mosi_pin,
            .miso_io_num = miso_pin,
            .sclk_io_num = sclk_pin,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 8,
        };
        gpio_set_pull_mode(miso_pin, GPIO_FLOATING);
        esp_err_t err = spi_bus_initialize((spi_host_device_t)spi_host, &bus_cfg, SPI_DMA_CH_AUTO);
        if (err != ESP_OK) {
            return err;
        }
        s_tmc2240_bus_initialized = true;
        s_tmc2240_bus_host = spi_host;
    } else if (s_tmc2240_bus_host != spi_host) {
        return ESP_ERR_INVALID_STATE;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = spi_clock_hz,
        .mode = 3,
        .spics_io_num = cs_pin,
        .queue_size = 2,
    };

    esp_err_t err = spi_bus_add_device((spi_host_device_t)spi_host, &dev_cfg, &ctx->spi_dev);
    if (err != ESP_OK) {
        return err;
    }

    ctx->spi_host = spi_host;
    ctx->initialized = true;
    ESP_LOGI(TAG, "TMC2240 %s SPI initialized (host=%d, cs=%d, sclk=%d, mosi=%d, miso=%d)",
             ctx->label, spi_host, cs_pin, sclk_pin, mosi_pin, miso_pin);
    return ESP_OK;
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

static void log_tmc2240_status(const char *label, tmc2240_spi_ctx_t *ctx)
{
    uint32_t gconf = 0;
    uint32_t ifcnt = 0;
    uint32_t chopconf = 0;
    uint32_t temp_raw = 0;
    float temp_c = NAN;

    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGW(TAG, "%s TMC2240 not initialized", label);
        return;
    }

    uint8_t tx[5] = {0};
    uint8_t rx_cmd[5] = {0};
    uint8_t rx_data[5] = {0};

    esp_err_t gconf_err = tmc2240_read_reg_raw_ctx(ctx, 0x00, tx, rx_cmd, rx_data);
    if (gconf_err == ESP_OK) {
        gconf = ((uint32_t)rx_data[1] << 24) |
                ((uint32_t)rx_data[2] << 16) |
                ((uint32_t)rx_data[3] << 8) |
                ((uint32_t)rx_data[4]);
    }

    esp_err_t ifcnt_err = tmc2240_read_reg_raw_ctx(ctx, 0x02, tx, rx_cmd, rx_data);
    if (ifcnt_err == ESP_OK) {
        ifcnt = ((uint32_t)rx_data[1] << 24) |
                ((uint32_t)rx_data[2] << 16) |
                ((uint32_t)rx_data[3] << 8) |
                ((uint32_t)rx_data[4]);
    }

    esp_err_t chopconf_err = tmc2240_read_reg_raw_ctx(ctx, 0x6C, tx, rx_cmd, rx_data);
    if (chopconf_err == ESP_OK) {
        chopconf = ((uint32_t)rx_data[1] << 24) |
                   ((uint32_t)rx_data[2] << 16) |
                   ((uint32_t)rx_data[3] << 8) |
                   ((uint32_t)rx_data[4]);
    }

    esp_err_t temp_err = tmc2240_read_reg_raw_ctx(ctx, 0x51, tx, rx_cmd, rx_data);
    if (temp_err == ESP_OK) {
        temp_raw = ((uint32_t)rx_data[1] << 24) |
                   ((uint32_t)rx_data[2] << 16) |
                   ((uint32_t)rx_data[3] << 8) |
                   ((uint32_t)rx_data[4]);
        temp_c = ((float)((uint16_t)(temp_raw & 0x00001FFF)) - 2038.0f) / 7.7f;
    }

    if (gconf_err == ESP_OK && ifcnt_err == ESP_OK && chopconf_err == ESP_OK && temp_err == ESP_OK) {
        ESP_LOGI(TAG, "%s TMC2240 status:", label);
        ESP_LOGI(TAG, "  GCONF   : 0x%08lX", (unsigned long)gconf);
        ESP_LOGI(TAG, "  IFCNT   : %lu", (unsigned long)ifcnt);
        ESP_LOGI(TAG, "  CHOPCONF: 0x%08lX", (unsigned long)chopconf);
        ESP_LOGI(TAG, "  TEMP    : %.2f C (raw 0x%08lX)", (double)temp_c, (unsigned long)temp_raw);
    } else {
        ESP_LOGW(TAG, "%s TMC2240 read failed (GCONF=%s IFCNT=%s CHOPCONF=%s TEMP=%s)",
                 label,
                 esp_err_to_name(gconf_err),
                 esp_err_to_name(ifcnt_err),
                 esp_err_to_name(chopconf_err),
                 esp_err_to_name(temp_err));
    }
}

void motor_module_init(motor_module_t *motor)
{
    stepper_channel_init(&motor->left);
    stepper_channel_init(&motor->right);
    motor_module_set_enabled(motor, false);
}

void motor_module_set_enabled(motor_module_t *motor, bool enable)
{
    motor->left.enabled = enable;
    motor->right.enabled = enable;
    set_enable_pin(&motor->left, enable);
    set_enable_pin(&motor->right, enable);
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
    gpio_set_level(motor->right.dir_pin, right_hz >= 0.0f ? 1 : 0);

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

    if (ch->last_toggle_us == 0 || (now_us - ch->last_toggle_us) >= half_period_us) {
        ch->step_level = !ch->step_level;
        gpio_set_level(ch->step_pin, ch->step_level ? 1 : 0);
        ch->last_toggle_us = now_us;
    }
}

void motor_module_service_step_pulses(motor_module_t *motor, int64_t now_us)
{
    service_channel(&motor->left, now_us);
    service_channel(&motor->right, now_us);
}

esp_err_t motor_module_tmc2240_spi_init(int spi_host,
                                        int sclk_pin,
                                        int mosi_pin,
                                        int miso_pin,
                                        int cs_pin,
                                        int spi_clock_hz)
{
    return tmc2240_spi_init_ctx(&s_tmc2240_left_ctx, spi_host, sclk_pin, mosi_pin, miso_pin, cs_pin, spi_clock_hz);
}

esp_err_t motor_module_tmc2240_right_spi_init(int spi_host,
                                              int sclk_pin,
                                              int mosi_pin,
                                              int miso_pin,
                                              int cs_pin,
                                              int spi_clock_hz)
{
    return tmc2240_spi_init_ctx(&s_tmc2240_right_ctx, spi_host, sclk_pin, mosi_pin, miso_pin, cs_pin, spi_clock_hz);
}

esp_err_t motor_module_tmc2240_write_reg(uint8_t reg_addr, uint32_t value)
{
    uint8_t tx[5] = {
        (uint8_t)(reg_addr | 0x80U),
        (uint8_t)(value >> 24),
        (uint8_t)(value >> 16),
        (uint8_t)(value >> 8),
        (uint8_t)(value),
    };
    uint8_t rx[5] = {0};

    return tmc2240_transfer_40b_ctx(&s_tmc2240_left_ctx, tx, rx);
}

esp_err_t motor_module_tmc2240_read_reg(uint8_t reg_addr, uint32_t *value_out)
{
    if (value_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t tx_cmd[5] = {0};
    uint8_t rx_cmd[5] = {0};
    uint8_t rx_data[5] = {0};
    esp_err_t err = tmc2240_read_reg_raw_ctx(&s_tmc2240_left_ctx, reg_addr, tx_cmd, rx_cmd, rx_data);
    if (err != ESP_OK) {
        return err;
    }

    *value_out = ((uint32_t)rx_data[1] << 24) |
                 ((uint32_t)rx_data[2] << 16) |
                 ((uint32_t)rx_data[3] << 8) |
                 ((uint32_t)rx_data[4]);
    return ESP_OK;
}

void motor_module_tmc2240_test_log(void)
{
    log_tmc2240_status("LEFT", &s_tmc2240_left_ctx);
}

void motor_module_tmc2240_right_test_log(void)
{
    log_tmc2240_status("RIGHT", &s_tmc2240_right_ctx);
}
