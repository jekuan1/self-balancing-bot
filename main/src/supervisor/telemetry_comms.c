#include "supervisor/telemetry_comms.h"

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "telemetry";

void telemetry_comms_init(telemetry_comms_t *telemetry)
{
    telemetry->publish_period_ms = 200;
    telemetry->last_publish_us = 0;
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
    ESP_LOGI(TAG, "state=%d tilt=%.2f deg rate=%.2f dps", (int)state, pose->tilt_deg, pose->tilt_rate_dps);
}
