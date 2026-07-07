#include "hal/motor_module.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

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
static SemaphoreHandle_t s_spi_read_mutex = NULL;
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
    esp_err_t err = spi_device_polling_transmit(ctx->spi_dev, &t);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s spi_device_polling_transmit failed: %s (tx=%02X%02X%02X%02X%02X)",
                 ctx->label, esp_err_to_name(err),
                 tx[0], tx[1], tx[2], tx[3], tx[4]);
    }
    return err;
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
static uint16_t tmc2240_extract_sg_result(uint32_t drv_status);

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

static uint8_t tmc2240_current_ma_to_cs(uint16_t current_ma, uint8_t current_range)
{
    // Full-scale RMS current is set by CURRENT_RANGE bits in DRV_CONF (0x0A).
    // CS must be scaled relative to the selected range's full scale or the
    // delivered current will be wrong (e.g. using 2300mA denominator with
    // range-0 hardware gives ~50% of the requested current).
    uint16_t max_rms_ma;
    if (current_range == 0) {
        max_rms_ma = 1100;
    } else if (current_range == 1) {
        max_rms_ma = 2300;
    } else {
        max_rms_ma = 3400;
    }

    float cs_f = ((float)current_ma / (float)max_rms_ma) * 32.0f - 1.0f;
    if (cs_f < 0.0f) return 0;
    if (cs_f > 31.0f) return 31;
    return (uint8_t)(cs_f + 0.5f);
}

static uint8_t tmc2240_select_current_range(uint16_t run_current_ma)
{
    if (run_current_ma < 1100U) {
        return 0x0;
    }
    if (run_current_ma < 2300U) {
        return 0x1;
    }
    return 0x2;
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

    // TMC2240 readback is delayed by one SPI frame.
    // This helper intentionally performs the address frame plus the dummy frame
    // so callers always get the value for the register they asked for.
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

    // Lock a mutex to make the two-frame read atomic across tasks. Without this,
    // another task can insert a write between the address frame and the NOP frame,
    // corrupting the TMC2240's internal read pointer and returning wrong data.
    // We use a FreeRTOS mutex instead of spi_device_acquire_bus because
    // acquire_bus is incompatible with spi_device_transmit/polling_transmit.
    if (s_spi_read_mutex != NULL) {
        xSemaphoreTake(s_spi_read_mutex, portMAX_DELAY);
    }

    // First transfer: send the read command. The TMC2240 uses a delayed-read
    // SPI protocol: the response to the read command is returned in the
    // *following* frame. The first transfer returns the status byte and the
    // result of the previous read; capture it in rx_cmd for debugging.
    esp_err_t err = tmc2240_transfer_40b_ctx(ctx, tx_cmd, rx_cmd);
    if (err != ESP_OK) {
        if (s_spi_read_mutex != NULL) xSemaphoreGive(s_spi_read_mutex);
        ESP_LOGW(TAG, "%s SPI read cmd transfer failed: %s", ctx->label, esp_err_to_name(err));
        return err;
    }

    // CSN high-time delay to meet TMC2240 inter-transfer timing requirement.
    esp_rom_delay_us(10);

    // Second transfer: send a NOP to clock out the data for the requested
    // register. rx_data[1..4] will contain the requested 32-bit register.
    uint8_t tx_nop[5] = {0, 0, 0, 0, 0};
    esp_err_t err2 = tmc2240_transfer_40b_ctx(ctx, tx_nop, rx_data);
    if (s_spi_read_mutex != NULL) xSemaphoreGive(s_spi_read_mutex);
    if (err2 != ESP_OK) {
        ESP_LOGW(TAG, "%s SPI read data transfer failed: %s", ctx->label, esp_err_to_name(err2));
        return err2;
    }

    if (reg_addr == 0x51) {
        ESP_LOGI(TAG, "%s reg=0x51 frame1=[%02X %02X %02X %02X %02X] frame2=[%02X %02X %02X %02X %02X]",
                 ctx->label,
                 rx_cmd[0], rx_cmd[1], rx_cmd[2], rx_cmd[3], rx_cmd[4],
                 rx_data[0], rx_data[1], rx_data[2], rx_data[3], rx_data[4]);
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

    if (s_spi_read_mutex == NULL) {
        s_spi_read_mutex = xSemaphoreCreateMutex();
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

    // DRV_STATUS (0x6F) bit assignments per TMC2240 datasheet.
    const bool stst  = (drv_status & (1UL << 31)) != 0;  // standstill
    const bool olb   = (drv_status & (1UL << 30)) != 0;  // open load B
    const bool ola   = (drv_status & (1UL << 29)) != 0;  // open load A
    const bool s2gb  = (drv_status & (1UL << 28)) != 0;  // short to GND B
    const bool s2ga  = (drv_status & (1UL << 27)) != 0;  // short to GND A
    const bool otpw  = (drv_status & (1UL << 26)) != 0;  // overtemp pre-warning
    const bool ot    = (drv_status & (1UL << 25)) != 0;  // overtemperature
    const bool s2vsb = (drv_status & (1UL << 24)) != 0;  // short to supply B
    const bool s2vsa = (drv_status & (1UL << 23)) != 0;  // short to supply A
    const uint16_t sg_result = tmc2240_extract_sg_result(drv_status); // bits 9:0
    const uint8_t cs_actual  = (uint8_t)((drv_status >> 16) & 0x1FU); // bits 20:16

    const bool any_fault = stst || olb || ola || s2gb || s2ga || otpw || ot || s2vsb || s2vsa;
    if (!any_fault && sg_result == 0 && cs_actual == 0) {
        snprintf(buf, buf_size, "no_fault_bits");
        return;
    }

    buf[0] = '\0';
    bool first = true;

    if (stst)  { snprintf(buf + strlen(buf), buf_size - strlen(buf), "%sSTST",  first ? "" : ","); first = false; }
    if (olb)   { snprintf(buf + strlen(buf), buf_size - strlen(buf), "%sOLB",   first ? "" : ","); first = false; }
    if (ola)   { snprintf(buf + strlen(buf), buf_size - strlen(buf), "%sOLA",   first ? "" : ","); first = false; }
    if (s2gb)  { snprintf(buf + strlen(buf), buf_size - strlen(buf), "%sS2GB",  first ? "" : ","); first = false; }
    if (s2ga)  { snprintf(buf + strlen(buf), buf_size - strlen(buf), "%sS2GA",  first ? "" : ","); first = false; }
    if (otpw)  { snprintf(buf + strlen(buf), buf_size - strlen(buf), "%sOTPW",  first ? "" : ","); first = false; }
    if (ot)    { snprintf(buf + strlen(buf), buf_size - strlen(buf), "%sOT",    first ? "" : ","); first = false; }
    if (s2vsb) { snprintf(buf + strlen(buf), buf_size - strlen(buf), "%sS2VSB", first ? "" : ","); first = false; }
    if (s2vsa) { snprintf(buf + strlen(buf), buf_size - strlen(buf), "%sS2VSA", first ? "" : ","); first = false; }

    snprintf(buf + strlen(buf), buf_size - strlen(buf), "%sSG=%u",    first ? "" : ",", (unsigned)sg_result);
    snprintf(buf + strlen(buf), buf_size - strlen(buf), ",CS_ACT=%u", (unsigned)cs_actual);
}

static uint16_t tmc2240_extract_sg_result(uint32_t drv_status)
{
    return (uint16_t)(drv_status & 0x03FFU);
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

    // DEBUG: Log raw register value once per context
    ESP_LOGI(TAG, "%s ADC_TEMP raw register: 0x%08lX | adc_raw: %u (0x%04X) | formula gives: %.1f°C",
             ctx->label, (unsigned long)temp_raw, adc_raw, adc_raw,
             ((float)adc_raw - 2038.0f) / 7.7f);

    // TMC2240 ADC_TEMP linearized conversion (typical):
    // T(C) = (ADC_RAW - 2038) / 7.7
    // Keep this as a typical calibration point unless board-level calibration
    // data is available for a more accurate offset.
    *out_temp_c = ((float)adc_raw - 2038.0f) / 7.7f;
    return ESP_OK;
}

static esp_err_t tmc2240_read_current_est_ma_ctx(tmc2240_spi_ctx_t *ctx, float *out_current_ma)
{
    if (ctx == NULL || out_current_ma == NULL) return ESP_ERR_INVALID_ARG;
    if (!ctx->initialized) return ESP_ERR_INVALID_STATE;

    uint32_t drv_status = 0;
    esp_err_t err = tmc2240_read_reg_u32_ctx(ctx, 0x6F, &drv_status);
    if (err != ESP_OK) return err;

    // CS_ACTUAL is a current scale, not direct amperes. Estimate RMS current
    // using the same 2.3A full-scale assumption used elsewhere in this file.
    const float max_rms_ma = 2300.0f;
    const float cs_actual = (float)((drv_status >> 16) & 0x1FU);
    *out_current_ma = ((cs_actual + 1.0f) / 32.0f) * max_rms_ma;
    return ESP_OK;
}

static esp_err_t tmc2240_read_pwm_diag_ctx(tmc2240_spi_ctx_t *ctx, uint32_t *pwm_status_out, uint32_t *pwm_auto_out)
{
    if (ctx == NULL || pwm_status_out == NULL) return ESP_ERR_INVALID_ARG;
    if (!ctx->initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t err = tmc2240_read_reg_u32_ctx(ctx, 0x71, pwm_status_out);
    if (err != ESP_OK) {
        return err;
    }

    if (pwm_auto_out != NULL) {
        err = tmc2240_read_reg_u32_ctx(ctx, 0x72, pwm_auto_out);
        if (err != ESP_OK) {
            return err;
        }
    }

    return ESP_OK;
}

void motor_module_init(motor_module_t *motor)
{
    stepper_channel_init(&motor->left);
    stepper_channel_init(&motor->right);
    set_enable_pin(&motor->left, true);
    set_enable_pin(&motor->right, true);
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

// NOTE: left and right share a single EN pin (GPIO 18), so enable=true just
// confirms the pin is driven active; there is no per-motor hardware gate.
// enable=false is purely a software pulse-stop — the TMC2240s remain powered
// and holding current at all times after motor_module_init().
void motor_module_set_enabled(motor_module_t *motor, bool enable)
{
    motor->left.enabled = enable;
    motor->right.enabled = enable;

    if (enable) {
        set_enable_pin(&motor->left, true);
        set_enable_pin(&motor->right, true);
    } else {
        motor->left.target_hz = 0.0f;
        motor->right.target_hz = 0.0f;
        motor->left.step_level = false;
        motor->right.step_level = false;
        motor->left.last_toggle_us = 0;
        motor->right.last_toggle_us = 0;
        if (motor->left.step_pin >= 0) gpio_set_level(motor->left.step_pin, 0);
        if (motor->right.step_pin >= 0) gpio_set_level(motor->right.step_pin, 0);
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

    // Clamp accumulated time to at most 2 half-periods to prevent step burst
    // when PID output jumps to a much higher frequency than the previous value.
    if ((now_us - ch->last_toggle_us) > 2 * half_period_us) {
        ch->last_toggle_us = now_us - 2 * half_period_us;
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
                                        int spi_clock_hz,
                                        bool right_ctx)
{
    tmc2240_spi_ctx_t *ctx = right_ctx ? &s_tmc2240_right_ctx : &s_tmc2240_left_ctx;
    return tmc2240_spi_init_ctx(ctx, spi_host, sclk_pin, mosi_pin, miso_pin, cs_pin, spi_clock_hz);
}

void motor_module_tmc2240_test_log(void)
{
    float left_temp = NAN;
    float right_temp = NAN;
    float left_current_ma = NAN;
    float right_current_ma = NAN;
    uint32_t left_pwm_status = 0;
    uint32_t right_pwm_status = 0;
    uint32_t left_pwm_auto = 0;
    uint32_t right_pwm_auto = 0;
    uint32_t left_drv_status = 0;
    uint32_t right_drv_status = 0;
    esp_err_t lerr = tmc2240_read_temp_c_ctx(&s_tmc2240_left_ctx, &left_temp);
    esp_err_t rerr = tmc2240_read_temp_c_ctx(&s_tmc2240_right_ctx, &right_temp);
    esp_err_t lcurr_err = tmc2240_read_current_est_ma_ctx(&s_tmc2240_left_ctx, &left_current_ma);
    esp_err_t rcurr_err = tmc2240_read_current_est_ma_ctx(&s_tmc2240_right_ctx, &right_current_ma);
    esp_err_t lpwm_err = tmc2240_read_pwm_diag_ctx(&s_tmc2240_left_ctx, &left_pwm_status, &left_pwm_auto);
    esp_err_t rpwm_err = tmc2240_read_pwm_diag_ctx(&s_tmc2240_right_ctx, &right_pwm_status, &right_pwm_auto);
    (void)tmc2240_read_reg_u32_ctx(&s_tmc2240_left_ctx, 0x6F, &left_drv_status);
    (void)tmc2240_read_reg_u32_ctx(&s_tmc2240_right_ctx, 0x6F, &right_drv_status);

    const bool left_ok = (lerr == ESP_OK && lcurr_err == ESP_OK && lpwm_err == ESP_OK);
    const bool right_ok = (rerr == ESP_OK && rcurr_err == ESP_OK && rpwm_err == ESP_OK);
    const uint16_t left_pwm_scale_sum = (uint16_t)(left_pwm_status & 0x03FFU);
    const uint16_t right_pwm_scale_sum = (uint16_t)(right_pwm_status & 0x03FFU);
    // SG_RESULT bits 9:0 — lower = higher load, ~0 = near stall
    const uint16_t left_sg  = tmc2240_extract_sg_result(left_drv_status);
    const uint16_t right_sg = tmc2240_extract_sg_result(right_drv_status);
    // STST bit (31) is set when motor is at standstill. SG_RESULT is
    // meaningless then — show "stst" instead of a misleading number.
    const bool left_stst  = (left_drv_status  & (1UL << 31)) != 0;
    const bool right_stst = (right_drv_status & (1UL << 31)) != 0;
    const uint8_t left_cs_actual  = (uint8_t)((left_drv_status  >> 16) & 0x1FU);
    const uint8_t right_cs_actual = (uint8_t)((right_drv_status >> 16) & 0x1FU);
    char left_sg_str[8], right_sg_str[8];
    if (left_stst)  { snprintf(left_sg_str,  sizeof(left_sg_str),  "stst"); }
    else            { snprintf(left_sg_str,  sizeof(left_sg_str),  "%u", (unsigned)left_sg); }
    if (right_stst) { snprintf(right_sg_str, sizeof(right_sg_str), "stst"); }
    else            { snprintf(right_sg_str, sizeof(right_sg_str), "%u", (unsigned)right_sg); }

    if (left_ok && right_ok) {
        ESP_LOGI(TAG,
                 "TMC — LEFT: %.2f C, %.0f mA, CS=%u, PWM_SUM=%u, AUTO=%u, SG=%s | RIGHT: %.2f C, %.0f mA, CS=%u, PWM_SUM=%u, AUTO=%u, SG=%s",
                 (double)left_temp,
                 (double)left_current_ma,
                 (unsigned)left_cs_actual,
                 (unsigned)left_pwm_scale_sum,
                 (unsigned)left_pwm_auto,
                 left_sg_str,
                 (double)right_temp,
                 (double)right_current_ma,
                 (unsigned)right_cs_actual,
                 (unsigned)right_pwm_scale_sum,
                 (unsigned)right_pwm_auto,
                 right_sg_str);
    } else if (left_ok) {
        ESP_LOGI(TAG,
                 "TMC — LEFT: %.2f C, %.0f mA, CS=%u, PWM_SUM=%u, AUTO=%u, SG=%s | RIGHT: ERR",
                 (double)left_temp,
                 (double)left_current_ma,
                 (unsigned)left_cs_actual,
                 (unsigned)left_pwm_scale_sum,
                 (unsigned)left_pwm_auto,
                 left_sg_str);
    } else if (right_ok) {
        ESP_LOGI(TAG,
                 "TMC — LEFT: ERR | RIGHT: %.2f C, %.0f mA, CS=%u, PWM_SUM=%u, AUTO=%u, SG=%s",
                 (double)right_temp,
                 (double)right_current_ma,
                 (unsigned)right_cs_actual,
                 (unsigned)right_pwm_scale_sum,
                 (unsigned)right_pwm_auto,
                 right_sg_str);
    } else {
        ESP_LOGW(TAG,
                 "TMC — LEFT: ERR(%s/%s/%s) | RIGHT: ERR(%s/%s/%s)",
                 esp_err_to_name(lerr),
                 esp_err_to_name(lcurr_err),
                 esp_err_to_name(lpwm_err),
                 esp_err_to_name(rerr),
                 esp_err_to_name(rcurr_err),
                 esp_err_to_name(rpwm_err));
    }
}

void motor_module_tmc2240_log_config(void)
{
    tmc2240_spi_ctx_t *ctxs[2] = {&s_tmc2240_left_ctx, &s_tmc2240_right_ctx};
    for (int i = 0; i < 2; i++) {
        tmc2240_spi_ctx_t *ctx = ctxs[i];
        if (!ctx->initialized) {
            ESP_LOGI(TAG, "%s TMC2240: not initialized", ctx->label);
            continue;
        }
        uint32_t drv_conf = 0, ihold_irun = 0, chopconf = 0;
        (void)tmc2240_read_reg_u32_ctx(ctx, 0x0A, &drv_conf);
        (void)tmc2240_read_reg_u32_ctx(ctx, 0x10, &ihold_irun);
        (void)tmc2240_read_reg_u32_ctx(ctx, 0x6C, &chopconf);
        const uint8_t current_range = (uint8_t)(drv_conf & 0x03U);
        const uint8_t adcinsel      = (uint8_t)((drv_conf >> 2) & 0x03U);
        const uint8_t ihold         = (uint8_t)(ihold_irun & 0x1FU);
        const uint8_t irun          = (uint8_t)((ihold_irun >> 8) & 0x1FU);
        const uint8_t mres          = (uint8_t)((chopconf >> 24) & 0x0FU);
        const bool    intpol        = (chopconf & (1UL << 28)) != 0;
        static const uint16_t mres_table[16] = {256,128,64,32,16,8,4,2,1,1,1,1,1,1,1,1};
        uint16_t usteps = mres_table[mres & 0x0F];
        static const uint16_t range_ma[3] = {1100, 2300, 3400};
        uint16_t full_scale = range_ma[current_range < 3 ? current_range : 2];
        uint16_t irun_ma  = (uint16_t)(((uint32_t)(irun  + 1) * full_scale) / 32);
        uint16_t ihold_ma = (uint16_t)(((uint32_t)(ihold + 1) * full_scale) / 32);
        ESP_LOGI(TAG,
                 "%s CONFIG: CURRENT_RANGE=%u(%umA fs) IRUN=%u(~%umA) IHOLD=%u(~%umA) MRES=%u(%u ustep) INTPOL=%d ADCINSEL=%u",
                 ctx->label,
                 current_range, full_scale,
                 irun, irun_ma,
                 ihold, ihold_ma,
                 mres, usteps,
                 intpol ? 1 : 0,
                 adcinsel);
    }
}

void motor_module_tmc2240_configure_robot_mode(const TMC2240_RobotConfig_t *config)
{
    TMC2240_RobotConfig_t cfg = {
        .run_current_ma = 500,        // Quiet driving current for the low-range setting
        .hold_current_ma = 100,       // Drop hold current to reduce standstill hiss
        .microsteps = 16,             // 16 microsteps with interpolation to 256 internally
        .interpolate = true,          // MUST BE TRUE for that "silky smooth" TMC motion
        .stealth_threshold = 500,     // Enable StealthChop for silent operation
        .stall_sensitivity = 0,       // Keep at 0 until you are ready for StallGuard
        .cool_step_enabled = false,   // Keep off until your current is higher
    };
    if (config != NULL) {
        cfg = *config;
    }

    const uint8_t mres = tmc2240_microsteps_to_mres(cfg.microsteps);
    const uint8_t current_range = tmc2240_select_current_range(cfg.run_current_ma);
    const uint8_t irun_cs = tmc2240_current_ma_to_cs(cfg.run_current_ma, current_range);
    const uint8_t ihold_cs = tmc2240_current_ma_to_cs(cfg.hold_current_ma, current_range);

    (void)cfg.cool_step_enabled;

    tmc2240_spi_ctx_t *ctxs[2] = {&s_tmc2240_right_ctx, &s_tmc2240_left_ctx};
    
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
        (void)tmc2240_write_reg_ctx(ctx, 0x01, 0x1F);

        // Explicitly enable stealthChop in GCONF (en_pwm_mode = bit 0).
        // Do a read-modify-write so we don't inadvertently clear other GCONF bits.
        uint32_t gconf = 0;
        if (tmc2240_read_reg_u32_ctx(ctx, 0x00, &gconf) == ESP_OK) {
            gconf |= (1U << 0); // set en_pwm_mode
            (void)tmc2240_write_reg_ctx(ctx, 0x00, gconf);
        } else {
            // If read fails, fall back to setting en_pwm_mode only.
            (void)tmc2240_write_reg_ctx(ctx, 0x00, 0x00000001);
        }

        // DRV_CONF (0x0A):
        //   bits 1:0 = CURRENT_RANGE (sets full-scale RMS current)
        //   bits 3:2 = ADCINSEL: 00=AIN/supply, 01=temperature sensor
        // Write CURRENT_RANGE first alone, then set ADCINSEL separately with a
        // delay between them. Some TMC2240 silicon ignores ADCINSEL when written
        // in the same frame as CURRENT_RANGE due to internal sequencing.
        const uint32_t drv_conf_range_only = (uint32_t)(current_range & 0x03U);
        (void)tmc2240_write_reg_ctx(ctx, 0x0A, drv_conf_range_only);
        esp_rom_delay_us(100);
        const uint32_t drv_conf_value = drv_conf_range_only | (1UL << 2);
        (void)tmc2240_write_reg_ctx(ctx, 0x0A, drv_conf_value);

        // Read back to verify. Retry once if ADCINSEL didn't land.
        esp_rom_delay_us(100);
        uint32_t drv_conf_read = 0;
        esp_err_t drv_conf_err = tmc2240_read_reg_u32_ctx(ctx, 0x0A, &drv_conf_read);
        if (drv_conf_err == ESP_OK && (drv_conf_read & 0x0FU) != (drv_conf_value & 0x0FU)) {
            esp_rom_delay_us(500);
            (void)tmc2240_write_reg_ctx(ctx, 0x0A, drv_conf_value);
            esp_rom_delay_us(200);
            (void)tmc2240_read_reg_u32_ctx(ctx, 0x0A, &drv_conf_read);
        }
        const uint32_t drv_conf_read_lower4 = drv_conf_read & 0x0FU;
        if (drv_conf_err != ESP_OK || drv_conf_read_lower4 != (drv_conf_value & 0x0FU)) {
            ESP_LOGE(TAG,
                     "%s CONFIG FAILED: Wrote DRV_CONF=0x%08lX, Read DRV_CONF=0x%08lX (%s)",
                     ctx->label,
                     (unsigned long)drv_conf_value,
                     (unsigned long)drv_conf_read,
                     esp_err_to_name(drv_conf_err));
        } else {
            ESP_LOGI(TAG,
                     "%s CONFIG SUCCESS: CURRENT_RANGE=%u ADCINSEL=temp (DRV_CONF=0x%08lX)",
                     ctx->label,
                     (unsigned)current_range,
                     (unsigned long)drv_conf_read);
        }

        // Configure Standstill Power Down timing: TPOWERDOWN (0x11)
        // This keeps the bridge powered while stepping stops, letting the
        // driver decay current gracefully instead of dropping the EN pin.
        (void)tmc2240_write_reg_ctx(ctx, 0x11, 10);

        // TPWMTHRS (0x13): StealthChop active for TSTEP >= this value (i.e. low speed).
        // Above this speed (TSTEP < TPWMTHRS) the driver switches to SpreadCycle.
        if (cfg.stealth_threshold > 0) {
            (void)tmc2240_write_reg_ctx(ctx, 0x13, cfg.stealth_threshold);
        }

        // TCOOLTHRS (0x14): StallGuard4 active when TSTEP <= this value.
        // TSTEP is the measured step period in internal clock cycles; at 500 Hz
        // steps TSTEP ≈ 24 000 — far above stealth_threshold=500. Setting
        // TCOOLTHRS = TPWMTHRS meant StallGuard was never active at robot speeds,
        // producing SG_RESULT=0 always. Set to 20-bit max so SG4 is always enabled.
        (void)tmc2240_write_reg_ctx(ctx, 0x14, 0xFFFFFUL);

        // Enable StealthChop2 auto-tuning and keep freewheel disabled so the
        // driver maintains holding torque using IHOLD at standstill.
        // PWM_FREQ=01 (~36.6kHz, above audible range) | PWM_AUTOSCALE | PWM_AUTOGRAD
        // PWM_OFS=36, PWM_GRAD=14 are TMC2240 datasheet recommended defaults for
        // good StealthChop AT#1 standstill tuning; zeroing them causes poor auto-tune.
        uint32_t pwmconf = (14UL << 8)   // PWM_GRAD = 14
                         | (36UL << 0)   // PWM_OFS  = 36
                         | (1UL  << 16)  // PWM_FREQ = 01 (~36.6 kHz)
                         | (1UL  << 18)  // PWM_AUTOSCALE
                         | (1UL  << 19); // PWM_AUTOGRAD
        (void)tmc2240_write_reg_ctx(ctx, 0x70, pwmconf);

        // Give the driver time to perform standstill tuning (AT#1) before motion.
        esp_rom_delay_us(130000);

        // SG4_THRS (0x74): StallGuard4 threshold + optional filter.
        // SG_RESULT < 2*SG4_THRS triggers a stall event on the DIAG pin.
        // SG4_FILT_EN (bit 8) enables averaging for more stable readings.
        // stall_sensitivity=0 disables stall signalling while still allowing
        // SG_RESULT reads for diagnostic purposes.
        {
            uint32_t sg4_thrs = ((uint32_t)(cfg.stall_sensitivity & 0xFF))
                              | (1UL << 8); // SG4_FILT_EN — smoother SG_RESULT
            (void)tmc2240_write_reg_ctx(ctx, 0x74, sg4_thrs);
        }

        // Set GLOBALSCALER to 0 (full scale / 256) for standard scaling.
        (void)tmc2240_write_reg_ctx(ctx, 0x0B, 0);

        // Configure operational parameters from the requested config.
        uint32_t chopconf = 0x10410150;
        chopconf |= ((uint32_t)mres << 24);
        chopconf |= 0x05U; // TOFF
        if (cfg.interpolate) {
            chopconf |= (1UL << 28); // INTPOL
        }
        (void)tmc2240_write_reg_ctx(ctx, 0x6C, chopconf);

        // Register format: bits 19:16=IHOLDDELAY, bits 12:8=IRUN, bits 4:0=IHOLD
        uint32_t ihold_irun = (6UL << 16) | (((uint32_t)irun_cs & 0x1F) << 8) | ((uint32_t)ihold_cs & 0x1F);
        (void)tmc2240_write_reg_ctx(ctx, 0x10, ihold_irun);

        // Register readback: verify SPI reads return plausible data.
        // CHOPCONF was just written, so it is a known-value check.
        // ADC_TEMP (0x51) bits 12:0 should be ~2231 at 25°C.
        // DRV_STATUS (0x6F) bits 9:0 = SG_RESULT, bits 20:16 = CS_ACTUAL.
        {
            uint32_t chopconf_rb = 0, ihold_rb = 0, adc_temp_rb = 0, drv_status_rb = 0;
            (void)tmc2240_read_reg_u32_ctx(ctx, 0x6C, &chopconf_rb);
            (void)tmc2240_read_reg_u32_ctx(ctx, 0x10, &ihold_rb);
            (void)tmc2240_read_reg_u32_ctx(ctx, 0x51, &adc_temp_rb);
            (void)tmc2240_read_reg_u32_ctx(ctx, 0x6F, &drv_status_rb);
            ESP_LOGI(TAG,
                     "%s RAW READBACK: CHOPCONF=0x%08lX IHOLD_IRUN=0x%08lX ADC_TEMP=0x%08lX DRV_STATUS=0x%08lX",
                     ctx->label,
                     (unsigned long)chopconf_rb,
                     (unsigned long)ihold_rb,
                     (unsigned long)adc_temp_rb,
                     (unsigned long)drv_status_rb);
        }

        ESP_LOGI(TAG,
                 "%s TMC2240 config: microsteps=%u IRUN_CS=%u IHOLD_CS=%u RANGE=%u INTPOL=%d TPWMTHRS=%lu TCOOLTHRS=0xFFFFF SG4_THRS=%u SG4_FILT=1",
                 ctx->label,
                 (unsigned)cfg.microsteps,
                 (unsigned)irun_cs,
                 (unsigned)ihold_cs,
                 (unsigned)current_range,
                 cfg.interpolate ? 1 : 0,
                 (unsigned long)cfg.stealth_threshold,
                 (unsigned)cfg.stall_sensitivity);
    }
}

void motor_module_tmc2240_freeze_tuning(void)
{
    // After the boot motor test, AT#1 has fully converged and PWM_AUTO (0x72)
    // holds the learned OFS_AUTO and GRAD_AUTO values.  Write those back into
    // PWMCONF as fixed OFS/GRAD and clear AUTOGRAD (bit 19) so AT#1 never
    // re-runs on subsequent start commands.  AUTOSCALE (bit 18) stays enabled
    // so amplitude still adapts to changing load.
    tmc2240_spi_ctx_t *ctxs[2] = {&s_tmc2240_left_ctx, &s_tmc2240_right_ctx};
    for (int i = 0; i < 2; i++) {
        tmc2240_spi_ctx_t *ctx = ctxs[i];
        if (!ctx->initialized) continue;

        uint32_t pwm_auto = 0;
        if (tmc2240_read_reg_u32_ctx(ctx, 0x72, &pwm_auto) != ESP_OK) {
            ESP_LOGW(TAG, "%s: failed to read PWM_AUTO; AUTOGRAD left enabled", ctx->label);
            continue;
        }
        const uint8_t ofs_auto  = (uint8_t)(pwm_auto & 0xFFU);
        const uint8_t grad_auto = (uint8_t)((pwm_auto >> 16) & 0xFFU);

        uint32_t pwmconf = 0;
        if (tmc2240_read_reg_u32_ctx(ctx, 0x70, &pwmconf) != ESP_OK) {
            ESP_LOGW(TAG, "%s: failed to read PWMCONF; AUTOGRAD left enabled", ctx->label);
            continue;
        }

        // Replace initial OFS/GRAD with AT#1-learned values and clear AUTOGRAD.
        pwmconf = (pwmconf & ~(0xFFUL | (0xFFUL << 8) | (1UL << 19)))
                | (uint32_t)ofs_auto
                | ((uint32_t)grad_auto << 8);
        (void)tmc2240_write_reg_ctx(ctx, 0x70, pwmconf);

        ESP_LOGI(TAG, "%s: AT#1 tuning frozen — OFS=%u GRAD=%u AUTOGRAD disabled",
                 ctx->label, ofs_auto, grad_auto);
    }
}

