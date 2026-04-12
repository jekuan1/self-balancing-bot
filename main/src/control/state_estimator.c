#include "control/state_estimator.h"

void state_estimator_init(state_estimator_t *estimator)
{
    estimator->last_tilt_deg = 0.0f;
    estimator->last_ts_us = 0;
    estimator->initialized = false;
}

void state_estimator_update(state_estimator_t *estimator, const imu_sample_t *sample, robot_pose_t *out_pose)
{
    out_pose->tilt_deg = sample->pitch_deg;
    out_pose->timestamp_us = sample->timestamp_us;

    if (!estimator->initialized || estimator->last_ts_us == 0) {
        out_pose->tilt_rate_dps = 0.0f;
        estimator->initialized = true;
    } else {
        float dt_s = (float)(sample->timestamp_us - estimator->last_ts_us) / 1000000.0f;
        if (dt_s <= 0.0f) {
            dt_s = 0.001f;
        }
        out_pose->tilt_rate_dps = (sample->pitch_deg - estimator->last_tilt_deg) / dt_s;
    }

    estimator->last_tilt_deg = sample->pitch_deg;
    estimator->last_ts_us = sample->timestamp_us;
}
