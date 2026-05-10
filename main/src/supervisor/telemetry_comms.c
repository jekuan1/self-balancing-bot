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

void telemetry_comms_publish(telemetry_comms_t *telemetry,
                             robot_state_t state,
                             const robot_pose_t *pose,
                             float target_pitch_deg,
                             float dynamic_target_pitch_deg,
                             float velocity_gain,
                             float control_output_hz,
                             float max_step_hz,
                             float left_step_hz,
                             float right_step_hz,
                             bool control_active)
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

    float saturation_pct = 0.0f;
    if (max_step_hz > 0.0f) {
        float abs_output = control_output_hz < 0.0f ? -control_output_hz : control_output_hz;
        saturation_pct = (abs_output / max_step_hz) * 100.0f;
        if (saturation_pct > 100.0f) {
            saturation_pct = 100.0f;
        }
    }

    // 2. Send over UDP as CSV:
    // yaw,pitch,roll,tilt_rate,lin_accel_x,target,dynamic_target,k_vel_p,output_hz,max_hz,saturation_pct,left_hz,right_hz,active
    if (s_udp_sock >= 0) {
        char payload[208];
        int len = snprintf(payload, sizeof(payload), "%.2f,%.2f,%.2f,%.2f,%.3f,%.2f,%.2f,%.3f,%.1f,%.1f,%.1f,%.1f,%.1f,%d\n",
                           pose->yaw_deg,
                           pose->pitch_deg,
                           pose->roll_deg,
                           pose->tilt_rate_dps,
                           pose->lin_accel_x,
                           target_pitch_deg,
                           dynamic_target_pitch_deg,
                           velocity_gain,
                           control_output_hz,
                           max_step_hz,
                           saturation_pct,
                           left_step_hz,
                           right_step_hz,
                           control_active ? 1 : 0);

        sendto(s_udp_sock, payload, len, 0, (struct sockaddr *)&s_dest_addr, sizeof(s_dest_addr));
    }
}
