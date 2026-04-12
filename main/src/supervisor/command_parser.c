#include "supervisor/command_parser.h"

void command_parser_init(command_parser_t *parser)
{
    parser->max_drive_velocity = 1.0f;
    parser->max_turn_rate = 1.0f;
}

bool command_parser_poll(command_parser_t *parser, control_setpoint_t *setpoint)
{
    (void)parser;
    (void)setpoint;
    return false;
}
