#include "hal/motor_module.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "motor_module";

typedef struct {
    spi_device_handle_t spi_dev;
    int spi_host;
    bool initialized;
    const char *label;
    int miso_pin;
    bool miso_checked;
} tmc2240_spi_ctx_t;

static tmc2240_spi_ctx_t s_tmc2240_left_ctx = {
    .spi_dev = NULL,
    .spi_host = -1,
    .initialized = false,
    .label = "LEFT",
    .miso_pin = -1,
    .miso_checked = false,
};

static tmc2240_spi_ctx_t s_tmc2240_right_ctx = {
    .spi_dev = NULL,
    .spi_host = -1,
    .initialized = false,
    .label = "RIGHT",
    .miso_pin = -1,
    .miso_checked = false,
};

static bool s_tmc2240_bus_initialized = false;
static int s_tmc2240_bus_host = -1;
static motor_module_t *s_active_motor = NULL;
static esp_timer_handle_t s_motor_timer_handle = NULL;

static void motor_timer_callback(void *arg)
{
    motor_module_t *motor = (motor_module_t *)arg;
    if (motor == NULL) {
        return;
    }

    int64_t now_us = esp_timer_get_time();
    motor_module_service_step_pulses(motor, now_us);
}

static uint32_t tmc2240_u32_from_frame_data(const uint8_t rx_data[5])
{
    return ((uint32_t)rx_data[1] << 24) |
           ((uint32_t)rx_data[2] << 16) |
           ((uint32_t)rx_data[3] << 8) |
           ((uint32_t)rx_data[4]);
}

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

// Forward declarations for functions defined later but used earlier.
static esp_err_t tmc2240_read_reg_raw_ctx(tmc2240_spi_ctx_t *ctx,
                                          uint8_t reg_addr,
                                          uint8_t tx_cmd[5],
                                          uint8_t rx_cmd[5],
                                          uint8_t rx_data[5]);
static void tmc2240_miso_sanity_check_once(tmc2240_spi_ctx_t *ctx);
static void tmc2240_format_gstat(uint8_t gstat, char *buf, size_t buf_size);
static void tmc2240_format_drv_status(uint32_t drv_status, char *buf, size_t buf_size);

static uint8_t tmc2240_microsteps_to_mres(uint16_t microsteps)
{
    if (microsteps == 256U) return 0x0;
    if (microsteps == 128U) return 0x1;
    if (microsteps == 64U) return 0x2;
    if (microsteps == 32U) return 0x3;
    if (microsteps == 16U) return 0x4;
    if (microsteps == 8U) return 0x5;
    if (microsteps == 4U) return 0x6;
    if (microsteps == 2U) return 0x7;
    if (microsteps == 1U) return 0x8;
    return 0x6;
}

static uint8_t tmc2240_current_ma_to_cs(uint16_t current_ma)
{
    // Approximate current scaling using the same equation as TMCStepper.
    const float rsense = 0.11f;
    float cs_f = (32.0f * 1.41421f * ((float)current_ma / 1000.0f) * (rsense + 0.02f) / 0.325f) - 1.0f;
    if (cs_f < 0.0f) {
        return 0;
    }
    if (cs_f > 31.0f) {
        return 31;
    }
    return (uint8_t)(cs_f + 0.5f);
}


// Convenience: write a 32-bit register to a specific TMC2240 context.
static esp_err_t tmc2240_write_reg_ctx(tmc2240_spi_ctx_t *ctx, uint8_t reg_addr, uint32_t value)
{
    if (ctx == NULL) return ESP_ERR_INVALID_ARG;
    uint8_t tx[5] = {
        (uint8_t)(reg_addr | 0x80U),
        (uint8_t)(value >> 24),
        (uint8_t)(value >> 16),
        (uint8_t)(value >> 8),
        (uint8_t)(value),
    };
    uint8_t rx[5] = {0};
    return tmc2240_transfer_40b_ctx(ctx, tx, rx);
}

// Convenience: read a 32-bit register from a specific TMC2240 context.
static esp_err_t tmc2240_read_reg_u32_ctx(tmc2240_spi_ctx_t *ctx, uint8_t reg_addr, uint32_t *value_out)
{
    if (ctx == NULL || value_out == NULL) return ESP_ERR_INVALID_ARG;
    uint8_t tx_cmd[5] = {0};
    uint8_t rx_cmd[5] = {0};
    uint8_t rx_data[5] = {0};
    esp_err_t err = tmc2240_read_reg_raw_ctx(ctx, reg_addr, tx_cmd, rx_cmd, rx_data);
    if (err != ESP_OK) return err;
    *value_out = tmc2240_u32_from_frame_data(rx_data);
    return ESP_OK;
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
        ESP_LOGW(TAG, "%s SPI read cmd transfer failed: %s", ctx->label, esp_err_to_name(err));
        return err;
    }

    // Small delay to ensure SPI bus state cleanup between transactions
    esp_rom_delay_us(10);

    uint8_t tx_nop[5] = {0, 0, 0, 0, 0};
    esp_err_t err2 = tmc2240_transfer_40b_ctx(ctx, tx_nop, rx_data);
    if (err2 != ESP_OK) {
        ESP_LOGW(TAG, "%s SPI read data transfer failed: %s", ctx->label, esp_err_to_name(err2));
        return err2;
    }

    return ESP_OK;
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
        // Keep MISO defined while still allowing the external driver to override it.
        // A floating MISO line is a common cause of "stable garbage" readback.
        gpio_set_direction(miso_pin, GPIO_MODE_INPUT);
        gpio_set_pull_mode(miso_pin, GPIO_PULLDOWN_ONLY);
        esp_err_t err = spi_bus_initialize((spi_host_device_t)spi_host, &bus_cfg, SPI_DMA_CH_AUTO);
        if (err != ESP_OK) {
            return err;
        }
        s_tmc2240_bus_initialized = true;
        s_tmc2240_bus_host = spi_host;
    } else if (s_tmc2240_bus_host != spi_host) {
        return ESP_ERR_INVALID_STATE;
    }

    // Use mode 3 directly, matching the earlier working configuration.
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = spi_clock_hz,
        .mode = 3,
        .spics_io_num = cs_pin,
        .queue_size = 2,
    };

    if (ctx->spi_dev != NULL) {
        (void)spi_bus_remove_device(ctx->spi_dev);
        ctx->spi_dev = NULL;
    }

    esp_err_t err = spi_bus_add_device((spi_host_device_t)spi_host, &dev_cfg, &ctx->spi_dev);
    if (err != ESP_OK) {
        return err;
    }

    ctx->spi_host = spi_host;
    ctx->initialized = true;
    ctx->miso_pin = miso_pin;

    // Perform one-time MISO floatiness check (useful for debugging wiring/CS issues)
    tmc2240_miso_sanity_check_once(ctx);

    ESP_LOGI(TAG, "TMC2240 %s SPI initialized (host=%d, cs=%d, sclk=%d, mosi=%d, miso=%d)",
             ctx->label, spi_host, cs_pin, sclk_pin, mosi_pin, miso_pin);
    return ESP_OK;
}

static void tmc2240_miso_sanity_check_once(tmc2240_spi_ctx_t *ctx)
{
    if (ctx == NULL || ctx->miso_pin < 0 || ctx->miso_checked) {
        return;
    }
    ctx->miso_checked = true;

    // If MISO is floating, changing the internal pull often changes the "received" bytes.
    uint8_t tx_nop[5] = {0, 0, 0, 0, 0};
    uint8_t rx_pd[5] = {0};
    uint8_t rx_pu[5] = {0};

    gpio_set_pull_mode(ctx->miso_pin, GPIO_PULLDOWN_ONLY);
    esp_rom_delay_us(50);
    int miso_level_pd = gpio_get_level(ctx->miso_pin);
    (void)tmc2240_transfer_40b_ctx(ctx, tx_nop, rx_pd);

    gpio_set_pull_mode(ctx->miso_pin, GPIO_PULLUP_ONLY);
    esp_rom_delay_us(50);
    int miso_level_pu = gpio_get_level(ctx->miso_pin);
    (void)tmc2240_transfer_40b_ctx(ctx, tx_nop, rx_pu);

    gpio_set_pull_mode(ctx->miso_pin, GPIO_PULLDOWN_ONLY);

    // Count byte differences between PD and PU reads.
    int diff_count = 0;
    for (int i = 0; i < 5; ++i) {
        if (rx_pd[i] != rx_pu[i]) ++diff_count;
    }

    // Only warn if the received frames actually differ; a sole GPIO-level
    // change is noisy on some platforms and doesn't indicate bus corruption
    // if the frame bytes are identical.
    if (diff_count > 0) {
        ESP_LOGW(TAG,
                 "%s MISO sanity: gpio_level PD=%d PU=%d | PD=%02X %02X %02X %02X %02X | PU=%02X %02X %02X %02X %02X",
                 ctx->label,
                 miso_level_pd,
                 miso_level_pu,
                 rx_pd[0], rx_pd[1], rx_pd[2], rx_pd[3], rx_pd[4],
                 rx_pu[0], rx_pu[1], rx_pu[2], rx_pu[3], rx_pu[4]);
        ESP_LOGW(TAG,
                 "%s If PD and PU differ a lot, MISO is likely floating (wrong CS/wiring).",
                 ctx->label);
    } else {
        ESP_LOGD(TAG,
                 "%s MISO sanity: gpio levels PD=%d PU=%d but frames identical; suppressing warning.",
                 ctx->label, miso_level_pd, miso_level_pu);
    }
}

static void tmc2240_format_gstat(uint8_t gstat, char *buf, size_t buf_size)
{
    if (buf == NULL || buf_size == 0) {
        return;
    }

    const bool reset = (gstat & (1U << 0)) != 0;
    const bool drv_err = (gstat & (1U << 1)) != 0;
    const bool uv_cp = (gstat & (1U << 2)) != 0;
    const uint8_t unknown = (uint8_t)(gstat & (uint8_t)~0x07U);

    if (!reset && !drv_err && !uv_cp && unknown == 0) {
        snprintf(buf, buf_size, "OK");
        return;
    }

    buf[0] = '\0';
    bool first = true;

    if (reset) {
        snprintf(buf + strlen(buf), buf_size - strlen(buf), "%sRESET", first ? "" : ",");
        first = false;
    }
    if (drv_err) {
        snprintf(buf + strlen(buf), buf_size - strlen(buf), "%sDRV_ERR", first ? "" : ",");
        first = false;
    }
    if (uv_cp) {
        snprintf(buf + strlen(buf), buf_size - strlen(buf), "%sUV_CP", first ? "" : ",");
    }

    if (unknown != 0) {
        snprintf(buf + strlen(buf), buf_size - strlen(buf), "%sUNK_0x%02X", first ? "" : ",", unknown);
    }
}

static void tmc2240_format_drv_status(uint32_t drv_status, char *buf, size_t buf_size)
{
    if (buf == NULL || buf_size == 0) {
        return;
    }

    const bool stst = (drv_status & (1UL << 31)) != 0;
    const bool olb = (drv_status & (1UL << 30)) != 0;
    const bool ola = (drv_status & (1UL << 29)) != 0;
    const bool s2gb = (drv_status & (1UL << 28)) != 0;
    const bool s2ga = (drv_status & (1UL << 27)) != 0;
    const bool otpw = (drv_status & (1UL << 26)) != 0;
    const bool ot = (drv_status & (1UL << 25)) != 0;

    if (!stst && !olb && !ola && !s2gb && !s2ga && !otpw && !ot) {
        snprintf(buf, buf_size, "no_fault_bits");
        return;
    }

    buf[0] = '\0';
    bool first = true;

    if (stst) {
        snprintf(buf + strlen(buf), buf_size - strlen(buf), "%sSTST", first ? "" : ",");
        first = false;
    }
    if (olb) {
        snprintf(buf + strlen(buf), buf_size - strlen(buf), "%sOLB", first ? "" : ",");
        first = false;
    }
    if (ola) {
        snprintf(buf + strlen(buf), buf_size - strlen(buf), "%sOLA", first ? "" : ",");
        first = false;
    }
    if (s2gb) {
        snprintf(buf + strlen(buf), buf_size - strlen(buf), "%sS2GB", first ? "" : ",");
        first = false;
    }
    if (s2ga) {
        snprintf(buf + strlen(buf), buf_size - strlen(buf), "%sS2GA", first ? "" : ",");
        first = false;
    }
    if (otpw) {
        snprintf(buf + strlen(buf), buf_size - strlen(buf), "%sOTPW", first ? "" : ",");
        first = false;
    }
    if (ot) {
        snprintf(buf + strlen(buf), buf_size - strlen(buf), "%sOT", first ? "" : ",");
    }
}

static void stepper_channel_init(stepper_channel_t *ch)
{
    uint64_t mask = 0;
    if (ch->step_pin >= 0) mask |= (1ULL << ch->step_pin);
    if (ch->dir_pin  >= 0) mask |= (1ULL << ch->dir_pin);
    if (ch->en_pin   >= 0) mask |= (1ULL << ch->en_pin);

    if (mask != 0) {
        gpio_config_t cfg = {
            .pin_bit_mask = mask,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&cfg));
    }

    ch->enabled = false;
    ch->target_hz = 0.0f;
    ch->last_toggle_us = 0;
    ch->step_level = false;
    if (ch->step_pin >= 0) gpio_set_level(ch->step_pin, 0);
    if (ch->dir_pin  >= 0) gpio_set_level(ch->dir_pin, 0);
}

static void set_enable_pin(const stepper_channel_t *ch, bool enable)
{
    int level = 0;
    if (ch->en_active_low) {
        level = enable ? 0 : 1;
    } else {
        level = enable ? 1 : 0;
    }
    if (ch->en_pin >= 0) {
        gpio_set_level(ch->en_pin, level);
    }
}

// NOTE: detailed per-driver status/logging removed — use combined temperature
// read via `tmc2240_read_temp_c_ctx()` and the combined test log.

// Read temperature (C) from a TMC2240 context. Returns ESP_OK on success.
static esp_err_t tmc2240_read_temp_c_ctx(tmc2240_spi_ctx_t *ctx, float *out_temp_c)
{
    if (ctx == NULL || out_temp_c == NULL) return ESP_ERR_INVALID_ARG;
    if (!ctx->initialized) return ESP_ERR_INVALID_STATE;
    uint32_t temp_raw = 0;
    esp_err_t err = tmc2240_read_reg_u32_ctx(ctx, 0x51, &temp_raw);
    if (err != ESP_OK) return err;
    *out_temp_c = ((float)((uint16_t)(temp_raw & 0x00001FFF)) - 2038.0f) / 7.7f;
    return ESP_OK;
}

void motor_module_init(motor_module_t *motor)
{
    stepper_channel_init(&motor->left);
    stepper_channel_init(&motor->right);
    motor_module_set_enabled(motor, false);

    s_active_motor = motor;

    if (s_motor_timer_handle == NULL) {
        const esp_timer_create_args_t timer_args = {
            .callback = motor_timer_callback,
            .arg = (void *)motor,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "motor_timer",
        };

        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_motor_timer_handle));
        ESP_ERROR_CHECK(esp_timer_start_periodic(s_motor_timer_handle, 10));
    } else {
        ESP_ERROR_CHECK(esp_timer_stop(s_motor_timer_handle));
        ESP_ERROR_CHECK(esp_timer_delete(s_motor_timer_handle));
        s_motor_timer_handle = NULL;

        const esp_timer_create_args_t timer_args = {
            .callback = motor_timer_callback,
            .arg = (void *)motor,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "motor_timer",
        };

        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_motor_timer_handle));
        ESP_ERROR_CHECK(esp_timer_start_periodic(s_motor_timer_handle, 10));
    }
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

    if (motor->left.dir_pin >= 0) gpio_set_level(motor->left.dir_pin, left_hz >= 0.0f ? 1 : 0);
    if (motor->right.dir_pin >= 0) gpio_set_level(motor->right.dir_pin, right_hz >= 0.0f ? 1 : 0);

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
        if (ch->step_pin >= 0) gpio_set_level(ch->step_pin, ch->step_level ? 1 : 0);
        ch->last_toggle_us += half_period_us;
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
    return tmc2240_write_reg_ctx(&s_tmc2240_left_ctx, reg_addr, value);
}

esp_err_t motor_module_tmc2240_read_reg(uint8_t reg_addr, uint32_t *value_out)
{
    if (value_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return tmc2240_read_reg_u32_ctx(&s_tmc2240_left_ctx, reg_addr, value_out);
}

void motor_module_tmc2240_test_log(void)
{
    float left_temp = NAN;
    float right_temp = NAN;
    esp_err_t lerr = tmc2240_read_temp_c_ctx(&s_tmc2240_left_ctx, &left_temp);
    esp_err_t rerr = tmc2240_read_temp_c_ctx(&s_tmc2240_right_ctx, &right_temp);

    if (lerr == ESP_OK && rerr == ESP_OK) {
        ESP_LOGI(TAG, "TMC TEMP — LEFT: %.2f C | RIGHT: %.2f C", (double)left_temp, (double)right_temp);
    } else if (lerr == ESP_OK) {
        ESP_LOGI(TAG, "TMC TEMP — LEFT: %.2f C | RIGHT: ERR", (double)left_temp);
    } else if (rerr == ESP_OK) {
        ESP_LOGI(TAG, "TMC TEMP — LEFT: ERR | RIGHT: %.2f C", (double)right_temp);
    } else {
        ESP_LOGW(TAG, "TMC TEMP — LEFT: ERR(%s) | RIGHT: ERR(%s)", esp_err_to_name(lerr), esp_err_to_name(rerr));
    }
}

void motor_module_tmc2240_configure_robot_mode(const TMC2240_RobotConfig_t *config)
{
    TMC2240_RobotConfig_t cfg = {
        .run_current_ma = 1200,
        .hold_current_ma = 350,
        .microsteps = 4,
        .interpolate = false,
        .stealth_threshold = 0,
        .stall_sensitivity = 0,
        .cool_step_enabled = false,
    };
    if (config != NULL) {
        cfg = *config;
    }

    const uint8_t mres = tmc2240_microsteps_to_mres(cfg.microsteps);
    const uint8_t irun_cs = tmc2240_current_ma_to_cs(cfg.run_current_ma);
    const uint8_t ihold_cs = tmc2240_current_ma_to_cs(cfg.hold_current_ma);

    // Keep these fields in API for future tuning; not yet wired to board logic.
    (void)cfg.stall_sensitivity;
    (void)cfg.cool_step_enabled;

    tmc2240_spi_ctx_t *ctxs[2] = {&s_tmc2240_left_ctx, &s_tmc2240_right_ctx};
    
    for (int i = 0; i < 2; i++) {
        tmc2240_spi_ctx_t *ctx = ctxs[i];
        if (!ctx->initialized) continue;
        uint8_t gstat = 0;
        uint8_t tx_gstat[5] = {0x01, 0, 0, 0, 0}; // GSTAT (Read)
        uint8_t rx_gstat[5] = {0};
        tmc2240_transfer_40b_ctx(ctx, tx_gstat, rx_gstat);
        gstat = rx_gstat[4];

        uint8_t rx_data[5] = {0};
        uint8_t tx_status[5] = {0x6F, 0, 0, 0, 0}; // DRV_STATUS (Read)
        tmc2240_transfer_40b_ctx(ctx, tx_status, rx_data);
        uint32_t drv_status = ((uint32_t)rx_data[1] << 24) | ((uint32_t)rx_data[2] << 16) | 
                             ((uint32_t)rx_data[3] << 8) | (uint32_t)rx_data[4];

        char gstat_text[48];
        char drv_text[96];
        tmc2240_format_gstat(gstat, gstat_text, sizeof(gstat_text));
        tmc2240_format_drv_status(drv_status, drv_text, sizeof(drv_text));

        ESP_LOGI(TAG, "%s TMC2240: GSTAT=0x%02X [%s] DRV_STATUS=0x%08lX [%s]",
                 ctx->label,
                 gstat,
                 gstat_text,
                 (unsigned long)drv_status,
                 drv_text);

        if (gstat & (1U << 2)) {
            ESP_LOGW(TAG, "%s TMC2240: UV_CP set in GSTAT", ctx->label);
        }

        // Configure operational parameters from the requested config.
        uint32_t chopconf = 0x10410150;
        chopconf |= ((uint32_t)mres << 24);
        chopconf |= 0x05U; // TOFF
        if (cfg.interpolate) {
            chopconf |= (1UL << 28); // INTPOL
        }
        uint8_t tx_chop[5] = { (uint8_t)(0x6C | 0x80), (uint8_t)(chopconf>>24), (uint8_t)(chopconf>>16), (uint8_t)(chopconf>>8), (uint8_t)chopconf };
        tmc2240_transfer_40b_ctx(ctx, tx_chop, rx_data);
        
        // Register format: bits 23:16=IRUN, bits 15:8=IHOLD, bits 7:5=IHOLDDELAY
        uint32_t ihold_irun = ((uint32_t)irun_cs << 16) | ((uint32_t)ihold_cs << 8) | (6U << 5);
        uint8_t tx_curr[5] = { (uint8_t)(0x10 | 0x80), (uint8_t)(ihold_irun>>24), (uint8_t)(ihold_irun>>16), (uint8_t)(ihold_irun>>8), (uint8_t)ihold_irun };
        tmc2240_transfer_40b_ctx(ctx, tx_curr, rx_data);

        if (cfg.stealth_threshold > 0) {
            uint8_t tx_tpwmthrs[5] = {
                (uint8_t)(0x13 | 0x80),
                (uint8_t)(cfg.stealth_threshold >> 24),
                (uint8_t)(cfg.stealth_threshold >> 16),
                (uint8_t)(cfg.stealth_threshold >> 8),
                (uint8_t)cfg.stealth_threshold,
            };
            tmc2240_transfer_40b_ctx(ctx, tx_tpwmthrs, rx_data);
        }

        ESP_LOGI(TAG,
                 "%s TMC2240 config: microsteps=%u IRUN_CS=%u IHOLD_CS=%u INTPOL=%d TPWMTHRS=%lu",
                 ctx->label,
                 (unsigned)cfg.microsteps,
                 (unsigned)irun_cs,
                 (unsigned)ihold_cs,
                 cfg.interpolate ? 1 : 0,
                 (unsigned long)cfg.stealth_threshold);
    }
}
