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
    pid->deriv_filter_state = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->prev_ts_us = 0;
}

float pid_controller_step(pid_controller_t *pid,
                          float setpoint,
                          float measurement,
                          float measured_rate_dps,
                          int64_t now_us)
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
    
    // Anti-windup: clamp the integrator to prevent runaway
    pid->integrator = clampf(pid->integrator, pid->out_min / (pid->ki > 0 ? pid->ki : 1.0f), pid->out_max / (pid->ki > 0 ? pid->ki : 1.0f));

    // Use the measured gyro rate directly instead of numerically differentiating
    // the angle error. This avoids extra noise from finite differences.
    pid->deriv_filter_state += 0.3f * (measured_rate_dps - pid->deriv_filter_state);

    float output = (pid->kp * error) + (pid->ki * pid->integrator) + (pid->kd * pid->deriv_filter_state);
    output = clampf(output, pid->out_min, pid->out_max);

    pid->prev_error = error;
    pid->prev_ts_us = now_us;
    return output;
}

void pid_controller_reset(pid_controller_t *pid)
{
    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
    pid->deriv_filter_state = 0.0f;
    pid->prev_ts_us = 0;
}
