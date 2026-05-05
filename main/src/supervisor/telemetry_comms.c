#include "supervisor/telemetry_comms.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include <stdio.h>

static const char *TAG = "telemetry";
static int s_udp_sock = -1;
static struct sockaddr_in s_dest_addr;

void telemetry_comms_init(telemetry_comms_t *telemetry)
{
    telemetry->publish_period_ms = 20; // Faster update for WiFi (50Hz)
    telemetry->last_publish_us = 0;

    // Create UDP socket
    s_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_udp_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return;
    }

    // Enable broadcast
    int broadcast = 1;
    setsockopt(s_udp_sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    // Set destination to broadcast address on the AP network
    s_dest_addr.sin_addr.s_addr = inet_addr("192.168.4.255");
    s_dest_addr.sin_family = AF_INET;
    s_dest_addr.sin_port = htons(1234); // Port 1234
}

void telemetry_comms_publish(telemetry_comms_t *telemetry, robot_state_t state, const robot_pose_t *pose)
{
    int64_t now_us = esp_timer_get_time();
    if (telemetry->last_publish_us != 0) {
        int64_t elapsed_us = now_us - telemetry->last_publish_us;
        if (elapsed_us < (int64_t)telemetry->publish_period_ms * 1000) {
            return;
        }
    }
    telemetry->last_publish_us = now_us;

    (void)state;

    // 1. Log to console as before
    ESP_LOGD(TAG, "Y=%.2f P=%.2f R=%.2f", pose->yaw_deg, pose->pitch_deg, pose->roll_deg);

    // 2. Send over UDP as CSV: "yaw,pitch,roll,tilt_rate"
    if (s_udp_sock >= 0) {
        char payload[80];
        int len = snprintf(payload, sizeof(payload), "%.2f,%.2f,%.2f,%.2f\n",
                           pose->yaw_deg,
                           pose->pitch_deg,
                           pose->roll_deg,
                           pose->tilt_rate_dps);

        sendto(s_udp_sock, payload, len, 0, (struct sockaddr *)&s_dest_addr, sizeof(s_dest_addr));
    }
}