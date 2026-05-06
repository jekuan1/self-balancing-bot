#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ROBOT_STATE_LOCKED = 0,
    ROBOT_STATE_READY,
    ROBOT_STATE_FAILURE,
} robot_state_t;

typedef struct {
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float gyro_pitch_dps;
    float lin_accel_x;
    int64_t timestamp_us;
} imu_sample_t;

typedef struct {
    float yaw_deg;
    float pitch_deg;
    float roll_deg;
    float tilt_deg;
    float tilt_rate_dps;
    float lin_accel_x;
    int64_t timestamp_us;
} robot_pose_t;

typedef struct {
    float left_step_hz;
    float right_step_hz;
} motor_command_t;

typedef struct {
    float drive_velocity;
    float turn_rate;
    float tilt_setpoint_deg;
} control_setpoint_t;

#endif