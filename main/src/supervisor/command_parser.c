#include "supervisor/command_parser.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <unistd.h>

#include "esp_log.h"
#include "robot_control.h"

static const char *TAG = "command_parser";

// ==============================================================================
// Utility Functions
// ==============================================================================

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

// Parse a line into argc/argv
static int parse_command_line(char *line, char **argv, int max_argc)
{
    if (line == NULL || argv == NULL || max_argc <= 0) {
        return 0;
    }

    int argc = 0;
    char *token = strtok(line, " \t");
    while (token != NULL && argc < max_argc) {
        argv[argc++] = token;
        token = strtok(NULL, " \t");
    }
    return argc;
}

// ==============================================================================
// Command Handlers
// ==============================================================================

static void handle_start(void)
{
    bool ok = robot_control_send_start();
    ESP_LOGI(TAG, "cmd=start -> %s", ok ? "accepted" : "rejected");
}

static void handle_stop(void)
{
    bool ok = robot_control_send_stop();
    ESP_LOGI(TAG, "cmd=stop -> %s", ok ? "accepted" : "rejected");
}

static bool parse_float_arg(const char *text, float *value)
{
    if (text == NULL || value == NULL) {
        return false;
    }

    char *endptr = NULL;
    float parsed = strtof(text, &endptr);
    if (endptr == text || (endptr != NULL && *endptr != '\0')) {
        return false;
    }

    *value = parsed;
    return true;
}

static void handle_tune_pid(int argc, char **argv)
{
    if (argc < 5 || strcmp(argv[1], "pid") != 0) {
        ESP_LOGW(TAG, "Usage: tune pid <kp> <ki> <kd>");
        return;
    }

    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    if (!parse_float_arg(argv[2], &kp) ||
        !parse_float_arg(argv[3], &ki) ||
        !parse_float_arg(argv[4], &kd)) {
        ESP_LOGW(TAG, "PID gains must be numeric");
        return;
    }

    if (robot_control_tune_pid(kp, ki, kd)) {
        ESP_LOGI(TAG, "cmd=tune pid -> kp=%.3f ki=%.3f kd=%.3f", kp, ki, kd);
    } else {
        ESP_LOGW(TAG, "PID tune rejected");
    }
}

static void handle_forward(int argc, char **argv)
{
    if (argc < 2) {
        ESP_LOGW(TAG, "Usage: forward <speed [0-100]>");
        return;
    }
    int speed = atoi(argv[1]);
    if (speed < 0 || speed > 100) {
        ESP_LOGW(TAG, "Speed must be 0-100");
        return;
    }
    float normalized = speed / 100.0f;
    ESP_LOGI(TAG, "cmd=forward speed=%.2f", normalized);
    // TODO: Implement drive_velocity setpoint
}

static void handle_backward(int argc, char **argv)
{
    if (argc < 2) {
        ESP_LOGW(TAG, "Usage: backward <speed [0-100]>");
        return;
    }
    int speed = atoi(argv[1]);
    if (speed < 0 || speed > 100) {
        ESP_LOGW(TAG, "Speed must be 0-100");
        return;
    }
    float normalized = speed / 100.0f;
    ESP_LOGI(TAG, "cmd=backward speed=%.2f", normalized);
    // TODO: Implement drive_velocity setpoint
}

static void handle_left(int argc, char **argv)
{
    if (argc < 2) {
        ESP_LOGW(TAG, "Usage: left <speed [0-100]>");
        return;
    }
    int speed = atoi(argv[1]);
    if (speed < 0 || speed > 100) {
        ESP_LOGW(TAG, "Speed must be 0-100");
        return;
    }
    float normalized = speed / 100.0f;
    ESP_LOGI(TAG, "cmd=left speed=%.2f", normalized);
    // TODO: Implement turn_rate setpoint
}

static void handle_right(int argc, char **argv)
{
    if (argc < 2) {
        ESP_LOGW(TAG, "Usage: right <speed [0-100]>");
        return;
    }
    int speed = atoi(argv[1]);
    if (speed < 0 || speed > 100) {
        ESP_LOGW(TAG, "Speed must be 0-100");
        return;
    }
    float normalized = speed / 100.0f;
    ESP_LOGI(TAG, "cmd=right speed=%.2f", normalized);
    // TODO: Implement turn_rate setpoint
}

static void handle_set_target(int argc, char **argv)
{
    if (argc < 3 || strcmp(argv[1], "target") != 0) {
        ESP_LOGW(TAG, "Usage: set target <pitch_deg>");
        return;
    }
    float pitch = 0.0f;
    if (!parse_float_arg(argv[2], &pitch)) {
        ESP_LOGW(TAG, "Pitch must be numeric");
        return;
    }
    robot_control_set_target(pitch);
}

static void handle_calibrate(void)
{
    float current = robot_control_get_pitch();
    robot_control_set_target(current);
    ESP_LOGI(TAG, "Calibrated: target pitch set to current reading (%.3f deg)", current);
}

static void handle_watchdog_clear(void)
{
    ESP_LOGI(TAG, "cmd=watchdog_clear (use 'stop' to recover from fault)");
    // TODO: Implement watchdog latch clear
}

static void handle_help(void)
{
    ESP_LOGI(TAG, "=== Available Commands ===");
    ESP_LOGI(TAG, "  start                    - Start balance control");
    ESP_LOGI(TAG, "  stop                     - Stop balance control");
    ESP_LOGI(TAG, "  tune pid <kp> <ki> <kd>  - Update PID gains live");
    ESP_LOGI(TAG, "  set target <pitch_deg>   - Set balance target pitch angle");
    ESP_LOGI(TAG, "  calibrate                - Set target to current pitch reading");
    ESP_LOGI(TAG, "  forward <0-100>          - Drive forward at speed");
    ESP_LOGI(TAG, "  backward <0-100>         - Drive backward at speed");
    ESP_LOGI(TAG, "  left <0-100>             - Turn left at speed");
    ESP_LOGI(TAG, "  right <0-100>            - Turn right at speed");
    ESP_LOGI(TAG, "  watchdog_clear           - Clear watchdog trip latch");
    ESP_LOGI(TAG, "  help                     - Show this help message");
}

// ==============================================================================
// Initialization
// ==============================================================================

void command_parser_init(command_parser_t *parser)
{
    if (parser == NULL) {
        return;
    }
    parser->max_drive_velocity = 1.0f;
    parser->max_turn_rate = 1.0f;
}

// ==============================================================================
// Polling (called from supervisor task at 20Hz)
// ==============================================================================

bool command_parser_poll(command_parser_t *parser, control_setpoint_t *setpoint)
{
    if (parser == NULL) {
        return false;
    }

    (void)setpoint;

    // Check if stdin has data available
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

    // Read line from stdin
    char line[256] = {0};
    if (fgets(line, sizeof(line), stdin) == NULL) {
        return false;
    }

    // Remove trailing newline
    size_t len = strlen(line);
    if (len > 0 && line[len - 1] == '\n') {
        line[len - 1] = '\0';
    }

    // Trim whitespace
    trim_ascii(line);

    if (line[0] == '\0') {
        return false;
    }

    // Parse command line
    char *argv[8] = {0};
    int argc = parse_command_line(line, argv, 8);

    if (argc == 0) {
        return false;
    }

    // Dispatch command
    if (strcmp(argv[0], "start") == 0) {
        handle_start();
    } else if (strcmp(argv[0], "stop") == 0) {
        handle_stop();
    } else if (strcmp(argv[0], "tune") == 0) {
        handle_tune_pid(argc, argv);
    } else if (strcmp(argv[0], "forward") == 0) {
        handle_forward(argc, argv);
    } else if (strcmp(argv[0], "backward") == 0) {
        handle_backward(argc, argv);
    } else if (strcmp(argv[0], "left") == 0) {
        handle_left(argc, argv);
    } else if (strcmp(argv[0], "right") == 0) {
        handle_right(argc, argv);
    } else if (strcmp(argv[0], "set") == 0) {
        handle_set_target(argc, argv);
    } else if (strcmp(argv[0], "calibrate") == 0) {
        handle_calibrate();
    } else if (strcmp(argv[0], "watchdog_clear") == 0) {
        handle_watchdog_clear();
    } else if (strcmp(argv[0], "help") == 0) {
        handle_help();
    } else {
        ESP_LOGW(TAG, "Unknown command: '%s' (try: help)", argv[0]);
    }

    return false;
}
