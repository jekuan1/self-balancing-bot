#ifndef MOTOR_MODULE_H
#define MOTOR_MODULE_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "robot_types.h"

typedef struct {
    int step_pin;
    int dir_pin;
    int en_pin;
    bool en_active_low;
    bool enabled;
    float target_hz;
    int64_t last_toggle_us;
    bool step_level;
} stepper_channel_t;

typedef struct {
    stepper_channel_t left;
    stepper_channel_t right;
    float max_step_hz;
} motor_module_t;

void motor_module_init(motor_module_t *motor);
void motor_module_set_enabled(motor_module_t *motor, bool enable);
void motor_module_apply_command(motor_module_t *motor, const motor_command_t *command);
void motor_module_service_step_pulses(motor_module_t *motor, int64_t now_us);

#endif