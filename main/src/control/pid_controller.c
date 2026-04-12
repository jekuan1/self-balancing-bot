#include "control/pid_controller.h"

static float clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

void pid_controller_init(pid_controller_t *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->prev_ts_us = 0;
}

float pid_controller_step(pid_controller_t *pid, float setpoint, float measurement, int64_t now_us)
{
    float error = setpoint - measurement;
    if (pid->prev_ts_us == 0) {
        pid->prev_ts_us = now_us;
        pid->prev_error = error;
        return clampf(pid->kp * error, pid->out_min, pid->out_max);
    }

    float dt_s = (float)(now_us - pid->prev_ts_us) / 1000000.0f;
    if (dt_s <= 0.0f) {
        dt_s = 0.001f;
    }

    pid->integrator += error * dt_s;
    float derivative = (error - pid->prev_error) / dt_s;

    float output = (pid->kp * error) + (pid->ki * pid->integrator) + (pid->kd * derivative);
    output = clampf(output, pid->out_min, pid->out_max);

    pid->prev_error = error;
    pid->prev_ts_us = now_us;
    return output;
}

void pid_controller_reset(pid_controller_t *pid)
{
    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_ts_us = 0;
}
