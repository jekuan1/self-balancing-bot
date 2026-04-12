#ifndef TELEMETRY_COMMS_H
#define TELEMETRY_COMMS_H

#include "robot_types.h"

typedef struct {
    uint32_t publish_period_ms;
    int64_t last_publish_us;
} telemetry_comms_t;

void telemetry_comms_init(telemetry_comms_t *telemetry);
void telemetry_comms_publish(telemetry_comms_t *telemetry, robot_state_t state, const robot_pose_t *pose);

#endif