#include "hal/motor_module.h"

#include <math.h>

#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "motor_module";

typedef struct {
    spi_device_handle_t spi_dev;
    int spi_host;
    bool initialized;
} tmc2240_spi_ctx_t;

static tmc2240_spi_ctx_t s_tmc2240_ctx = {
    .spi_dev = NULL,
    .spi_host = -1,
    .initialized = false,
};

static motor_module_t *s_active_motor = NULL;
static esp_timer_handle_t s_motor_timer_handle = NULL;

static void motor_timer_callback(void *arg)
{
    motor_module_t *motor = (motor_module_t *)arg;
    int64_t now_us = esp_timer_get_time();
    motor_module_service_step_pulses(motor, now_us);
}

static esp_err_t tmc2240_transfer_40b(uint8_t tx[5], uint8_t rx[5])
{
    if (!s_tmc2240_ctx.initialized || s_tmc2240_ctx.spi_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    spi_transaction_t t = {
        .length = 40,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    return spi_device_transmit(s_tmc2240_ctx.spi_dev, &t);
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
                                        int cs_pin,
                                        int ain_pin,
                                        int spi_clock_hz)
{
    if (s_tmc2240_ctx.initialized) {
        return ESP_OK;
    }

    if (ain_pin >= 0) {
        gpio_config_t ain_cfg = {
            .pin_bit_mask = 1ULL << ain_pin,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&ain_cfg));
        gpio_set_level(ain_pin, 1); // Drive high by default
    }

    if (spi_clock_hz <= 0) {
        spi_clock_hz = 1000000;
    }

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = mosi_pin,
        .miso_io_num = miso_pin,
        .sclk_io_num = sclk_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 8,
    };
    esp_err_t err = spi_bus_initialize((spi_host_device_t)spi_host, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        return err;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = spi_clock_hz,
        .mode = 3,
        .spics_io_num = cs_pin,
        .queue_size = 2,
    };

    err = spi_bus_add_device((spi_host_device_t)spi_host, &dev_cfg, &s_tmc2240_ctx.spi_dev);
    if (err != ESP_OK) {
        (void)spi_bus_free((spi_host_device_t)spi_host);
        return err;
    }

    s_tmc2240_ctx.spi_host = spi_host;
    s_tmc2240_ctx.initialized = true;
    ESP_LOGI(TAG, "TMC2240 SPI test initialized (host=%d, cs=%d, sclk=%d, mosi=%d, miso=%d)",
             spi_host, cs_pin, sclk_pin, mosi_pin, miso_pin);
    return ESP_OK;
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

    return tmc2240_transfer_40b(tx, rx);
}

esp_err_t motor_module_tmc2240_read_reg(uint8_t reg_addr, uint32_t *value_out)
{
    if (value_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Trinamic SPI returns register data one frame later.
    uint8_t tx_cmd[5] = { (uint8_t)(reg_addr & 0x7FU), 0, 0, 0, 0 };
    uint8_t rx_cmd[5] = {0};
    esp_err_t err = tmc2240_transfer_40b(tx_cmd, rx_cmd);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t tx_nop[5] = {0, 0, 0, 0, 0};
    uint8_t rx_data[5] = {0};
    err = tmc2240_transfer_40b(tx_nop, rx_data);
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
    uint32_t gconf = 0;
    uint32_t ifcnt = 0;
    uint32_t chopconf = 0;
    float temp_c = 0.0f;

    uint8_t cs_actual = 0;
    esp_err_t gconf_err = motor_module_tmc2240_read_reg(0x00, &gconf);
    esp_err_t ifcnt_err = motor_module_tmc2240_read_reg(0x02, &ifcnt);
    esp_err_t chop_err = motor_module_tmc2240_read_reg(0x6C, &chopconf);
    (void)motor_module_tmc2240_get_temp(&temp_c);
    (void)motor_module_tmc2240_get_current_scaling(&cs_actual);

    if (gconf_err == ESP_OK && ifcnt_err == ESP_OK && chop_err == ESP_OK) {
        // Estimate mA: (CS+1)/32 * IFS(3.0A) * 0.707 (RMS)
        float est_ma = ((float)cs_actual + 1.0f) / 32.0f * 3000.0f * 0.707f;
        ESP_LOGI(TAG, "TMC2240 GCONF=0x%08lX IFCNT=0x%08lX CHOP=0x%08lX TEMP=%.1f C CS=%d (~%.0fmA)",
                 (unsigned long)gconf,
                 (unsigned long)ifcnt,
                 (unsigned long)chopconf,
                 temp_c,
                 (int)cs_actual,
                 est_ma);
        
        if (temp_c > 75.0f) {
            ESP_LOGW(TAG, "WARNING: TMC2240 High Temperature (>75C)!");
        }
    } else {
        ESP_LOGW(TAG, "TMC2240 read failed (GCONF=%s IFCNT=%s)",
                 esp_err_to_name(gconf_err),
                 esp_err_to_name(ifcnt_err));
    }
}
void motor_module_tmc2240_configure(void)
{
    // CHOPCONF: toff=3, hstrt=4, hend=1, tbl=2, mres=4 (16 microsteps)
    // MRES is bits 24:27. 4 = 16 microsteps.
    uint32_t chopconf = 0x040100C3; 
    motor_module_tmc2240_write_reg(0x6C, chopconf);

    // ihold_irun: IHOLD=8, IRUN=8, IHOLDDELAY=6
    // Lowered IRUN to 8 for safe breadboard/low-power testing.
    uint32_t ihold_irun = (6 << 16) | (8 << 8) | (8);
    motor_module_tmc2240_write_reg(0x10, ihold_irun);

    // GCONF: en_pwm_mode=1 (StealthChop), shaft=0
    motor_module_tmc2240_write_reg(0x00, 0x00000004);

    // PWMCONF: stealthchop defaults
    motor_module_tmc2240_write_reg(0x70, 0xC40C001E);

    ESP_LOGI(TAG, "TMC2240 low-power configuration applied (IRUN=8, IHOLD=8)");
}
void motor_module_tmc2240_configure_robot_mode(void)
{
    // 1. GLOBAL_SCALER (0x0B): Set to 255 (Full Scale) to ensure the driver is utilizing 
    // its full internal current range.
    motor_module_tmc2240_write_reg(0x0B, 0x000000FF);

    // 2. DRV_CONF (0x0A): Set to use the internal 2.0V reference for current regulation.
    motor_module_tmc2240_write_reg(0x0A, 0x00010000);

    // 3. CHOPCONF (0x6C): toff=3, hstrt=4, hend=1, tbl=2, mres=4 (16 microsteps)
    // hend=1 (Hysteresis End = -2) provides a more stable current ripple in SpreadCycle.
    uint32_t chopconf = 0x040100C3; 
    motor_module_tmc2240_write_reg(0x6C, chopconf);

    // 4. IHOLD_IRUN (0x10): IHOLD=6, IRUN=14, IHOLDDELAY=6
    // IRUN=14 provides approx. 450mA peak (approx. 320mA RMS) on BTT modules.
    uint32_t ihold_irun = (6 << 16) | (14 << 8) | (6);
    motor_module_tmc2240_write_reg(0x10, ihold_irun);

    // 5. GCONF: en_pwm_mode=0 (SpreadCycle), drv_ext_res=1 (External Resistors), multistep_filt=1
    motor_module_tmc2240_write_reg(0x00, 0x0000000A);

    // 6. PWMCONF: Disable StealthChop autotuning since we are in SpreadCycle
    motor_module_tmc2240_write_reg(0x70, 0x00000000);

    ESP_LOGI(TAG, "TMC2240 High-Performance Robot Mode applied (BTT Config, IRUN=14)");
}

esp_err_t motor_module_tmc2240_get_temp(float *temp_c)
{
    if (temp_c == NULL) return ESP_ERR_INVALID_ARG;

    uint32_t data = 0;
    esp_err_t err = motor_module_tmc2240_read_reg(0x51, &data);
    if (err != ESP_OK) return err;

    // TMC2240 Temperature formula from sample code: (ADC_DATA - 2038) / 7.7
    *temp_c = (float)((uint16_t)(data & 0x1FFF) - 2038) / 7.7f;
    return ESP_OK;
}

esp_err_t motor_module_tmc2240_get_current_scaling(uint8_t *cs_actual)
{
    if (cs_actual == NULL) return ESP_ERR_INVALID_ARG;

    uint32_t data = 0;
    esp_err_t err = motor_module_tmc2240_read_reg(0x6F, &data); // DRV_STATUS
    if (err != ESP_OK) return err;

    // CS_ACTUAL is in bits [20:16]
    *cs_actual = (uint8_t)((data >> 16) & 0x1F);
    return ESP_OK;
}
