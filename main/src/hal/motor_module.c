#include "hal/motor_module.h"

#include <math.h>

#include "esp_err.h"

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
