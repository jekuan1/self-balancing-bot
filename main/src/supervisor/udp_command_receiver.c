#include "supervisor/udp_command_receiver.h"
#include "supervisor/command_parser.h"
#include "robot_control.h"

#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include "esp_log.h"
#include "lwip/sockets.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

bool robot_control_send_stop(void);
bool robot_control_send_start(void);

static const char *TAG = "udp_cmd_receiver";

// Reuse the same command handlers from command_parser
// (We'll dispatch raw command strings through the parser logic)

static void trim_ascii(char *s)
{
    if (s == NULL) return;
    
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

static void udp_command_task(void *pvParameters)
{
    (void)pvParameters;
    
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create UDP socket");
        vTaskDelete(NULL);
        return;
    }
    
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(5555);
    
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind UDP socket on port 5555");
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "UDP command receiver listening on port 5555");
    
    char buffer[256];
    struct sockaddr_in src_addr = {0};
    socklen_t src_addr_len = sizeof(src_addr);
    
    while (1) {
        int len = recvfrom(sock, buffer, sizeof(buffer) - 1, 0,
                          (struct sockaddr *)&src_addr, &src_addr_len);
        
        if (len > 0) {
            buffer[len] = '\0';
            trim_ascii(buffer);
            
            if (strlen(buffer) > 0) {
                ESP_LOGI(TAG, "UDP cmd from %s: %s",
                        inet_ntoa(src_addr.sin_addr), buffer);
                
                // Dispatch through the same parser logic as serial commands
                // Parse command and arguments
                char *argv[8] = {0};
                int argc = 0;
                
                char copy[256];
                strncpy(copy, buffer, sizeof(copy) - 1);
                copy[sizeof(copy) - 1] = '\0';
                
                char *token = strtok(copy, " \t");
                while (token != NULL && argc < 8) {
                    argv[argc++] = token;
                    token = strtok(NULL, " \t");
                }
                
                if (argc > 0) {
                    if (strcmp(argv[0], "start") == 0) {
                        robot_control_send_start();
                    } else if (strcmp(argv[0], "stop") == 0) {
                        robot_control_send_stop();
                    } else if (strcmp(argv[0], "tune") == 0 && argc >= 5 && strcmp(argv[1], "pid") == 0) {
                        float kp = 0.0f;
                        float ki = 0.0f;
                        float kd = 0.0f;
                        if (parse_float_arg(argv[2], &kp) &&
                            parse_float_arg(argv[3], &ki) &&
                            parse_float_arg(argv[4], &kd)) {
                            robot_control_tune_pid(kp, ki, kd);
                            ESP_LOGI(TAG, "tune pid -> kp=%.3f ki=%.3f kd=%.3f", kp, ki, kd);
                        } else {
                            ESP_LOGW(TAG, "Usage: tune pid <kp> <ki> <kd>");
                        }
                    } else if (strcmp(argv[0], "forward") == 0) {
                        if (argc >= 2) {
                            int speed = atoi(argv[1]);
                            if (speed >= 0 && speed <= 100) {
                                ESP_LOGI(TAG, "forward speed=%d", speed);
                            }
                        }
                    } else if (strcmp(argv[0], "backward") == 0) {
                        if (argc >= 2) {
                            int speed = atoi(argv[1]);
                            if (speed >= 0 && speed <= 100) {
                                ESP_LOGI(TAG, "backward speed=%d", speed);
                            }
                        }
                    } else if (strcmp(argv[0], "left") == 0) {
                        if (argc >= 2) {
                            int speed = atoi(argv[1]);
                            if (speed >= 0 && speed <= 100) {
                                ESP_LOGI(TAG, "left speed=%d", speed);
                            }
                        }
                    } else if (strcmp(argv[0], "right") == 0) {
                        if (argc >= 2) {
                            int speed = atoi(argv[1]);
                            if (speed >= 0 && speed <= 100) {
                                ESP_LOGI(TAG, "right speed=%d", speed);
                            }
                        }
                    } else if (strcmp(argv[0], "watchdog_clear") == 0) {
                        ESP_LOGI(TAG, "watchdog_clear");
                    } else {
                        ESP_LOGW(TAG, "Unknown command: %s", argv[0]);
                    }
                }
            }
        }
    }
    
    close(sock);
    vTaskDelete(NULL);
}

esp_err_t udp_command_receiver_init(void)
{
    BaseType_t result = xTaskCreate(udp_command_task,
                                     "udp_cmd_rcv",
                                     4096,
                                     NULL,
                                     3,
                                     NULL);
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UDP command receiver task");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}
