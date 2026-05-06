#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <stdbool.h>

typedef enum {
    ROBOT_CONTROL_CMD_NONE = 0,
    ROBOT_CONTROL_CMD_STOP = 1,
    ROBOT_CONTROL_CMD_START = 2,
    ROBOT_CONTROL_CMD_MOTOR_TEST = 3,
} robot_control_command_t;

typedef struct {
    float left_hz;   // 0 = don't move left
    float right_hz;  // 0 = don't move right
    uint32_t duration_ms;
} motor_test_params_t;

bool robot_control_send_command(robot_control_command_t command);
bool robot_control_send_stop(void);
bool robot_control_send_start(void);
bool robot_control_send_motor_test(const motor_test_params_t *params);
bool robot_control_tune_pid(float kp, float ki, float kd);
bool robot_control_set_target(float pitch_deg);
float robot_control_get_pitch(void);

#endif
