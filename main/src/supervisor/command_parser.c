#include "supervisor/command_parser.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <sys/select.h>
#include <unistd.h>

#include "esp_log.h"
#include "robot_control.h"

static const char *TAG = "command_parser";

static void trim_ascii(char *s)
{
    if (s == NULL) {
        return;
    }

    size_t len = strlen(s);
    while (len > 0 && isspace((unsigned char)s[len - 1])) {
        s[--len] = '\0';
    }

    size_t start = 0;
    while (s[start] != '\0' && isspace((unsigned char)s[start])) {
        start++;
    }
    if (start > 0) {
        memmove(s, s + start, strlen(s + start) + 1);
    }
}

static void lowercase_ascii(char *s)
{
    if (s == NULL) {
        return;
    }
    for (size_t i = 0; s[i] != '\0'; ++i) {
        s[i] = (char)tolower((unsigned char)s[i]);
    }
}

void command_parser_init(command_parser_t *parser)
{
    if (parser == NULL) {
        return;
    }
    parser->max_drive_velocity = 1.0f;
    parser->max_turn_rate = 1.0f;
}

bool command_parser_poll(command_parser_t *parser, control_setpoint_t *setpoint)
{
    if (parser == NULL) {
        return false;
    }

    (void)parser;
    (void)setpoint;

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);

    struct timeval timeout = {
        .tv_sec = 0,
        .tv_usec = 0,
    };

    int ready = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout);
    if (ready <= 0 || !FD_ISSET(STDIN_FILENO, &readfds)) {
        return false;
    }

    char line[64] = {0};
    if (fgets(line, sizeof(line), stdin) == NULL) {
        return false;
    }

    trim_ascii(line);
    lowercase_ascii(line);
    if (line[0] == '\0') {
        return false;
    }

    if (strcmp(line, "stop") == 0) {
        bool ok = robot_control_send_stop();
        ESP_LOGI(TAG, "cmd=stop -> %s", ok ? "accepted" : "rejected");
        return false;
    }

    if (strcmp(line, "start") == 0) {
        bool ok = robot_control_send_start();
        ESP_LOGI(TAG, "cmd=start -> %s", ok ? "accepted" : "rejected");
        return false;
    }

    if (strcmp(line, "help") == 0) {
        ESP_LOGI(TAG, "Commands: start, stop, help");
        return false;
    }

    ESP_LOGW(TAG, "Unknown command: '%s' (try: start, stop, help)", line);
    return false;
}
