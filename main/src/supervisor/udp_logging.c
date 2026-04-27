#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "esp_log.h"
#include "lwip/sockets.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static int s_udp_log_sock = -1;
static struct sockaddr_in s_dest_addr;
static QueueHandle_t s_log_queue = NULL;

typedef struct {
    char msg[256];
} log_msg_t;

// Background task that actually sends the UDP packets
static void udp_log_task(void *pvParameters)
{
    log_msg_t log;
    while (1) {
        if (xQueueReceive(s_log_queue, &log, portMAX_DELAY)) {
            int len = strlen(log.msg);
            if (s_udp_log_sock >= 0) {
                // Send to Broadcast
                sendto(s_udp_log_sock, log.msg, len, 0, (struct sockaddr *)&s_dest_addr, sizeof(s_dest_addr));
                
                // Also send directly to likely laptop IPs (.2 and .3)
                struct sockaddr_in direct_addr = s_dest_addr;
                direct_addr.sin_addr.s_addr = inet_addr("192.168.4.2");
                sendto(s_udp_log_sock, log.msg, len, 0, (struct sockaddr *)&direct_addr, sizeof(direct_addr));
                
                direct_addr.sin_addr.s_addr = inet_addr("192.168.4.3");
                sendto(s_udp_log_sock, log.msg, len, 0, (struct sockaddr *)&direct_addr, sizeof(direct_addr));
            }
        }
    }
}

// Vprintf replacement that pushes to the Queue
static int udp_vprintf(const char *fmt, va_list l)
{
    // Always print to Serial first
    vprintf(fmt, l);

    if (s_log_queue) {
        log_msg_t log;
        int len = vsnprintf(log.msg, sizeof(log.msg), fmt, l);
        if (len > 0) {
            // Non-blocking send to avoid hanging the caller
            xQueueSend(s_log_queue, &log, 0);
        }
    }
    
    return 0; // Return value not strictly checked by ESP-IDF log system
}

void udp_logging_printf(const char *fmt, ...)
{
    if (s_log_queue) {
        log_msg_t log;
        va_list args;
        va_start(args, fmt);
        int len = vsnprintf(log.msg, sizeof(log.msg), fmt, args);
        va_end(args);
        
        if (len > 0) {
            xQueueSend(s_log_queue, &log, 0);
        }
        
        // Also echo to Serial
        va_start(args, fmt);
        vprintf(fmt, args);
        va_end(args);
    }
}

void udp_logging_init(const char *ip_addr, int port)
{
    s_log_queue = xQueueCreate(20, sizeof(log_msg_t));
    if (!s_log_queue) return;

    s_udp_log_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_udp_log_sock < 0) return;

    // Enable Broadcast
    int broadcast = 1;
    setsockopt(s_udp_log_sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    memset(&s_dest_addr, 0, sizeof(s_dest_addr));
    s_dest_addr.sin_addr.s_addr = inet_addr(ip_addr);
    s_dest_addr.sin_family = AF_INET;
    s_dest_addr.sin_port = htons(port);

    // Create the background worker task
    xTaskCreate(udp_log_task, "udp_log_task", 4096, NULL, 5, NULL);

    // Hook into ESP-IDF logging system
    esp_log_set_vprintf(udp_vprintf);
}
