#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float integrator;
    float prev_error;
    float out_min;
    float out_max;
    int64_t prev_ts_us;
} pid_controller_t;

void pid_controller_init(pid_controller_t *pid, float kp, float ki, float kd, float out_min, float out_max);
float pid_controller_step(pid_controller_t *pid, float setpoint, float measurement, int64_t now_us);
void pid_controller_reset(pid_controller_t *pid);

#endif