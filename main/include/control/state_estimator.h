#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include "robot_types.h"

typedef struct {
    float last_tilt_deg;
    int64_t last_ts_us;
    bool initialized;
} state_estimator_t;

void state_estimator_init(state_estimator_t *estimator);
void state_estimator_update(state_estimator_t *estimator, const imu_sample_t *sample, robot_pose_t *out_pose);

#endif