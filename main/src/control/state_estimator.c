#include "control/state_estimator.h"

void state_estimator_init(state_estimator_t *estimator)
{
    estimator->last_tilt_deg = 0.0f;
    estimator->last_ts_us = 0;
    estimator->initialized = false;
}

void state_estimator_update(state_estimator_t *estimator, const imu_sample_t *sample, robot_pose_t *out_pose)
{
    out_pose->yaw_deg = sample->yaw_deg;
    out_pose->pitch_deg = sample->pitch_deg;
    out_pose->roll_deg = sample->roll_deg;
    out_pose->tilt_deg = sample->pitch_deg;
    out_pose->timestamp_us = sample->timestamp_us;
    
    // Direct usage of gyro for tilt rate - much cleaner than differentiation
    out_pose->tilt_rate_dps = sample->gyro_pitch_dps;

    estimator->last_tilt_deg = sample->pitch_deg;
    estimator->last_ts_us = sample->timestamp_us;
    estimator->initialized = true;
}
