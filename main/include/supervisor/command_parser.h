#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <stdbool.h>
#include "robot_types.h"

typedef struct {
    float max_drive_velocity;
    float max_turn_rate;
} command_parser_t;

void command_parser_init(command_parser_t *parser);
bool command_parser_poll(command_parser_t *parser, control_setpoint_t *setpoint);

#endif