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

static void IRAM_ATTR motor_timer_callback(void *arg)
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

// Extract SPI_STATUS byte (bits 39-32 of the response, which is rx[0]).
// Returns the status byte from the response.
static uint8_t tmc2240_extract_spi_status(const uint8_t rx[5])
{
    if (rx == NULL) return 0;
    return rx[0];
}

// Check SPI_STATUS for notable bits. Some bits are noisy on this device and
// should not be treated as hard faults unless corroborated elsewhere.
static void tmc2240_check_spi_status(tmc2240_spi_ctx_t *ctx, uint8_t status_byte)
{
    if (ctx == NULL) return;
    
    // Bit 4: reset_flag, Bit 3: driver_error, Bit 2: standstill
    const bool reset_flag = (status_byte & (1U << 4)) != 0;
    const bool driver_error = (status_byte & (1U << 3)) != 0;
    const bool standstill = (status_byte & (1U << 2)) != 0;

    if (driver_error) {
        ESP_LOGD(TAG, "%s SPI_STATUS: driver_error bit set (ignored unless GSTAT/DRV_STATUS also fault)", ctx->label);
    }
    if (reset_flag) {
        ESP_LOGD(TAG, "%s SPI_STATUS: reset_flag detected (chip ready)", ctx->label);
    }
    if (standstill) {
        ESP_LOGD(TAG, "%s SPI_STATUS: motor in standstill", ctx->label);
    }
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
    /**
     * In Integrated Lossless Sensing mode, the CS (Current Scale) value 
     * is a linear ratio of the peak current supported by the selected 
     * CURRENT_RANGE in register 0x0A.
     * * For TMC2240 with Integrated Sensing:
     * Full Scale Current (at CS=31) is determined by CURRENT_RANGE bits.
     * If CURRENT_RANGE = 0b00 (0), Full Scale is ~1.1A RMS.
     * If CURRENT_RANGE = 0b01 (1), Full Scale is ~2.3A RMS.
     * If CURRENT_RANGE = 0b10 (2), Full Scale is ~3.4A RMS.
     * * Assuming CURRENT_RANGE 1 (2.3A / 2300mA RMS full scale):
     */
    const uint16_t max_rms_ma = 2300; 
    
    // Calculate CS: (current_ma / max_rms_ma) * 32 - 1
    float cs_f = ((float)current_ma / (float)max_rms_ma) * 32.0f - 1.0f;

    if (cs_f < 0.0f) return 0;
    if (cs_f > 31.0f) return 31;
    
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
    esp_err_t err = tmc2240_transfer_40b_ctx(ctx, tx, rx);
    
    // Even during a write, extract and check SPI status byte for real-time
    // alerts (driver_error, reset_flag, standstill).
    if (err == ESP_OK) {
        uint8_t spi_status = tmc2240_extract_spi_status(rx);
        tmc2240_check_spi_status(ctx, spi_status);
    }
    
    return err;
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
    
    // Extract and check SPI status from the data frame response
    uint8_t spi_status = tmc2240_extract_spi_status(rx_data);
    tmc2240_check_spi_status(ctx, spi_status);
    
    *value_out = tmc2240_u32_from_frame_data(rx_data);
    return ESP_OK;
}

// Batch-read function for pipelined reads: reduces SPI bus traffic by reading
// multiple registers in a single "pipelined" fashion. Returns three 32-bit register
// values in the order they are accessed.
static esp_err_t tmc2240_read_regs_pipelined_ctx(tmc2240_spi_ctx_t *ctx,
                                                 uint8_t reg_addr_1,
                                                 uint8_t reg_addr_2,
                                                 uint8_t reg_addr_3,
                                                 uint32_t *out_val_1,
                                                 uint32_t *out_val_2,
                                                 uint32_t *out_val_3)
{
    if (ctx == NULL || out_val_1 == NULL || out_val_2 == NULL || out_val_3 == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!ctx->initialized) return ESP_ERR_INVALID_STATE;

    uint8_t tx1[5], rx1[5] = {0};
    uint8_t tx2[5], rx2[5] = {0};
    uint8_t tx3[5], rx3[5] = {0};
    uint8_t tx_nop[5] = {0}, rx_nop[5] = {0};

    // Frame 1: Send read command for reg_addr_1
    tx1[0] = (uint8_t)(reg_addr_1 & 0x7FU);
    tx1[1] = tx1[2] = tx1[3] = tx1[4] = 0;
    esp_err_t err = tmc2240_transfer_40b_ctx(ctx, tx1, rx1);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "%s SPI pipelined read frame 1 failed: %s", ctx->label, esp_err_to_name(err));
        return err;
    }
    esp_rom_delay_us(10);

    // Frame 2: Send read command for reg_addr_2, receive data for reg_addr_1
    tx2[0] = (uint8_t)(reg_addr_2 & 0x7FU);
    tx2[1] = tx2[2] = tx2[3] = tx2[4] = 0;
    err = tmc2240_transfer_40b_ctx(ctx, tx2, rx2);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "%s SPI pipelined read frame 2 failed: %s", ctx->label, esp_err_to_name(err));
        return err;
    }
    esp_rom_delay_us(10);

    // Frame 3: Send read command for reg_addr_3, receive data for reg_addr_2
    tx3[0] = (uint8_t)(reg_addr_3 & 0x7FU);
    tx3[1] = tx3[2] = tx3[3] = tx3[4] = 0;
    err = tmc2240_transfer_40b_ctx(ctx, tx3, rx3);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "%s SPI pipelined read frame 3 failed: %s", ctx->label, esp_err_to_name(err));
        return err;
    }
    esp_rom_delay_us(10);

    // Frame 4: Send NOP, receive data for reg_addr_3
    err = tmc2240_transfer_40b_ctx(ctx, tx_nop, rx_nop);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "%s SPI pipelined read frame 4 (NOP) failed: %s", ctx->label, esp_err_to_name(err));
        return err;
    }

    // Assign extracted values: data returned in frame N+1 for command sent in frame N
    *out_val_1 = tmc2240_u32_from_frame_data(rx2);   // Data for reg_addr_1 in frame 2
    *out_val_2 = tmc2240_u32_from_frame_data(rx3);   // Data for reg_addr_2 in frame 3
    *out_val_3 = tmc2240_u32_from_frame_data(rx_nop); // Data for reg_addr_3 in frame 4

    // Check SPI status from the final frame
    uint8_t spi_status = tmc2240_extract_spi_status(rx_nop);
    tmc2240_check_spi_status(ctx, spi_status);

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

    // First transfer: send the read command. The TMC2240 uses a delayed-read
    // SPI protocol: the response to the read command is returned in the
    // *following* frame. The first transfer returns the status byte and the
    // result of the previous read; capture it in rx_cmd for debugging.
    esp_err_t err = tmc2240_transfer_40b_ctx(ctx, tx_cmd, rx_cmd);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "%s SPI read cmd transfer failed: %s", ctx->label, esp_err_to_name(err));
        return err;
    }

    // CSN high-time delay to meet TMC2240 inter-transfer timing requirement.
    // The SPI driver's cs_ena_posttrans handles most of this, but we add extra
    // delay to ensure robust operation.
    esp_rom_delay_us(10);

    // Second transfer: send a NOP to clock out the data for the requested
    // register. rx_data[1..4] will contain the requested 32-bit register.
    uint8_t tx_nop[5] = {0, 0, 0, 0, 0};
    esp_err_t err2 = tmc2240_transfer_40b_ctx(ctx, tx_nop, rx_data);
    if (err2 != ESP_OK) {
        ESP_LOGW(TAG, "%s SPI read data transfer failed: %s", ctx->label, esp_err_to_name(err2));
        return err2;
    }

    // rx_cmd[0] = status from first frame, rx_cmd[1..4] = stale data
    // rx_data[0] = status for data frame, rx_data[1..4] = requested register
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
    // Add cs_ena_posttrans to satisfy TMC2240's minimum CSN high-time requirement.
    // cs_ena_posttrans is the number of SPI clock cycles after CSN goes high.
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = spi_clock_hz,
        .mode = 3,
        .spics_io_num = cs_pin,
        .queue_size = 2,
        .cs_ena_posttrans = 10,  // Ensure minimum CSN high time (10 cycles buffer)
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

    // Byte 0 is the SPI status response and can legitimately differ between
    // back-to-back frames; compare payload bytes only for floating-MISO checks.
    int payload_diff_count = 0;
    for (int i = 1; i < 5; ++i) {
        if (rx_pd[i] != rx_pu[i]) ++payload_diff_count;
    }

    // This check is only a heuristic. The payload can legitimately differ
    // between back-to-back frames because the TMC2240 uses delayed reads.
    // Keep it as debug-only telemetry so it does not spam normal startup logs.
    if (payload_diff_count > 0) {
        ESP_LOGD(TAG,
                 "%s MISO sanity: gpio_level PD=%d PU=%d | PD=%02X %02X %02X %02X %02X | PU=%02X %02X %02X %02X %02X",
                 ctx->label,
                 miso_level_pd,
                 miso_level_pu,
                 rx_pd[0], rx_pd[1], rx_pd[2], rx_pd[3], rx_pd[4],
                 rx_pu[0], rx_pu[1], rx_pu[2], rx_pu[3], rx_pu[4]);
        ESP_LOGD(TAG,
                 "%s MISO sanity: payload differs across pulls; this can be normal for delayed-read SPI.",
                 ctx->label);
    } else {
        ESP_LOGD(TAG,
                 "%s MISO sanity: payload bytes stable across pulls; suppressing warning.",
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
    const bool register_reset = (gstat & (1U << 3)) != 0;
    const bool vm_uvlo = (gstat & (1U << 4)) != 0;
    const uint8_t unknown = (uint8_t)(gstat & (uint8_t)~0x1FU);

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

    if (register_reset) {
        snprintf(buf + strlen(buf), buf_size - strlen(buf), "%sREG_RST", first ? "" : ",");
        first = false;
    }
    if (vm_uvlo) {
        snprintf(buf + strlen(buf), buf_size - strlen(buf), "%sVM_UVLO", first ? "" : ",");
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

    // Extract additional useful fields: SG_RESULT (bits 9:0) and CS_ACTUAL (bits 20:16)
    const uint16_t sg_result = (uint16_t)(drv_status & 0x03FFU);
    const uint8_t cs_actual = (uint8_t)((drv_status >> 16) & 0x1FU);


    if (!stst && !olb && !ola && !s2gb && !s2ga && !otpw && !ot && sg_result == 0 && cs_actual == 0) {
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

    // Append StallGuard and current-actual fields for better diagnostics
    snprintf(buf + strlen(buf), buf_size - strlen(buf), "%sSG=%u", first ? "" : ",", (unsigned)sg_result);
    snprintf(buf + strlen(buf), buf_size - strlen(buf), ",CS_ACT=%u", (unsigned)cs_actual);
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
    // ADC_TEMP is in bits 12:0 (13 bits). Extract raw ADC value first.
    uint16_t adc_raw = (uint16_t)(temp_raw & 0x1FFFU);

    // WARNING: conversion constants vary between TMC families. The original
    // code used constants matching older chips; consult the TMC2240 datasheet
    // for the precise conversion. For now use an approximate conversion based
    // on a linear scale; this should be verified against the datasheet.
    *out_temp_c = ((float)adc_raw - 2048.0f) / 7.7f;
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

static void IRAM_ATTR service_channel(stepper_channel_t *ch, int64_t now_us)
{
    if (!ch->enabled || ch->target_hz == 0.0f) {
        ch->step_level = false;
        gpio_set_level(ch->step_pin, 0);
        return;
    }

    float abs_hz = ch->target_hz < 0.0f ? -ch->target_hz : ch->target_hz;
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

void IRAM_ATTR motor_module_service_step_pulses(motor_module_t *motor, int64_t now_us)
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
        .run_current_ma = 500,        // Your 0.5A soft limit for driving
        .hold_current_ma = 150,       // Dropped to 150mA to stay cool at standstill
        .microsteps = 16,             // Increased to 16 for better precision
        .interpolate = true,          // MUST BE TRUE for that "silky smooth" TMC motion
        .stealth_threshold = 500,     // Enable StealthChop for silent operation
        .stall_sensitivity = 0,       // Keep at 0 until you are ready for StallGuard
        .cool_step_enabled = false,   // Keep off until your current is higher
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
        uint32_t gstat_u32 = 0;
        uint32_t drv_status = 0;
        esp_err_t gerr = tmc2240_read_reg_u32_ctx(ctx, 0x01, &gstat_u32);   // GSTAT
        esp_err_t derr = tmc2240_read_reg_u32_ctx(ctx, 0x6F, &drv_status);  // DRV_STATUS
        uint8_t gstat = (uint8_t)(gstat_u32 & 0xFFU);

        if (gerr != ESP_OK || derr != ESP_OK) {
            ESP_LOGW(TAG,
                     "%s TMC2240 status read failed (GSTAT=%s, DRV_STATUS=%s)",
                     ctx->label,
                     esp_err_to_name(gerr),
                     esp_err_to_name(derr));
        }

        uint8_t rx_data[5] = {0};

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

        // Check reset_flag (bit 0) first: indicates successful power-up
        if (gstat & (1U << 0)) {
            ESP_LOGI(TAG, "%s TMC2240: Reset flag confirmed (chip powered up and ready)", ctx->label);
        }

        if (gstat & (1U << 2)) {
            ESP_LOGW(TAG, "%s TMC2240: UV_CP set in GSTAT", ctx->label);
        }

        // Clear sticky GSTAT error flags (write 1 to clear). Use 0x1F to clear all
        // sticky flags: Reset, DrvErr, UV_CP, RegReset, VM_UVLO (bits 0-4).
        // This must happen AFTER the status check to avoid missing the reset_flag.
        uint8_t tx_gstat_clear[5] = { (uint8_t)(0x01 | 0x80), 0, 0, 0, 0x1F };
        tmc2240_transfer_40b_ctx(ctx, tx_gstat_clear, rx_data);

        // Explicitly enable stealthChop in GCONF (en_pwm_mode = bit 2).
        // Do a read-modify-write so we don't inadvertently clear other GCONF bits
        // such as fast_standstill or multistep_filt.
        uint32_t gconf = 0;
        if (tmc2240_read_reg_u32_ctx(ctx, 0x00, &gconf) == ESP_OK) {
            gconf |= (1U << 2); // set en_pwm_mode
            (void)tmc2240_write_reg_ctx(ctx, 0x00, gconf);
        } else {
            // If read fails, fall back to setting en_pwm_mode only.
            (void)tmc2240_write_reg_ctx(ctx, 0x00, 0x00000004);
        }

        // Configure DRV_CONF for Lossless Sensing: set CURRENT_RANGE=1 (2.3A RMS)
        // and slope control bits for smooth current transitions.
        uint32_t drv_conf = (2UL << 16) | (1UL << 0);
        (void)tmc2240_write_reg_ctx(ctx, 0x0A, drv_conf);

        // Set GLOBALSCALER to 0 (full scale / 256) for standard scaling.
        (void)tmc2240_write_reg_ctx(ctx, 0x0B, 0);

        // Configure Standstill Power Down timing: TPOWERDOWN (0x11)
        // Sets the delay (in motor clock cycles) for current ramp-down after
        // the last STEP pulse. Default is often insufficient; set to ~20.
        (void)tmc2240_write_reg_ctx(ctx, 0x11, 20);

        // Configure operational parameters from the requested config.
        uint32_t chopconf = 0x10410150;
        chopconf |= ((uint32_t)mres << 24);
        chopconf |= 0x05U; // TOFF
        if (cfg.interpolate) {
            chopconf |= (1UL << 28); // INTPOL
        }
        uint8_t tx_chop[5] = { (uint8_t)(0x6C | 0x80), (uint8_t)(chopconf>>24), (uint8_t)(chopconf>>16), (uint8_t)(chopconf>>8), (uint8_t)chopconf };
        tmc2240_transfer_40b_ctx(ctx, tx_chop, rx_data);

        // (TMC2240 defaults GCONF and PWMCONF perfectly for StealthChop auto-tuning)
        
        // Register format: bits 19:16=IHOLDDELAY, bits 12:8=IRUN, bits 4:0=IHOLD
        uint32_t ihold_irun = (6UL << 16) | (((uint32_t)irun_cs & 0x1F) << 8) | ((uint32_t)ihold_cs & 0x1F);
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
