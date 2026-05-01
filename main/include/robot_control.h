#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <stdbool.h>

typedef enum {
    ROBOT_CONTROL_CMD_NONE = 0,
    ROBOT_CONTROL_CMD_STOP = 1,
    ROBOT_CONTROL_CMD_START = 2,
} robot_control_command_t;

bool robot_control_send_command(robot_control_command_t command);
bool robot_control_send_stop(void);
bool robot_control_send_start(void);

#endif
