#include "hal/ota_module.h"
#include <string.h>
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_system.h"

static const char *TAG = "ota_module";

/* 
 * HTTP POST Handler for /update
 * This accepts a raw binary stream and writes it to the next OTA partition.
 */
static esp_err_t update_post_handler(httpd_req_t *req)
{
    esp_ota_handle_t update_handle = 0;
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);

    if (update_partition == NULL) {
        ESP_LOGE(TAG, "No OTA partition found!");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Starting OTA update to partition: %s", update_partition->label);

    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    char buf[1024];
    int remaining = req->content_len;
    int received;

    while (remaining > 0) {
        received = httpd_req_recv(req, buf, sizeof(buf));
        if (received <= 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            ESP_LOGE(TAG, "Fast flash failed: Socket error during recv");
            esp_ota_end(update_handle);
            return ESP_FAIL;
        }

        err = esp_ota_write(update_handle, buf, received);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed (%s)", esp_err_to_name(err));
            esp_ota_end(update_handle);
            return ESP_FAIL;
        }
        remaining -= received;

        // Log progress every ~10%
        static int last_percent = -1;
        int percent = (int)(((float)(req->content_len - remaining) / req->content_len) * 100);
        if (percent % 10 == 0 && percent != last_percent) {
            ESP_LOGI(TAG, "OTA Progress: %d%% (%d / %d bytes)", percent, (int)(req->content_len - remaining), (int)req->content_len);
            last_percent = percent;
        }
    }

    err = esp_ota_end(update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed (%s)", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA Update Success! Rebooting in 1 second...");
    httpd_resp_sendstr(req, "Update Success! Robot is rebooting...");

    // Delay reboot to allow HTTP response to send
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();

    return ESP_OK;
}

esp_err_t ota_module_init(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 8;
    config.stack_size = 8192;

    ESP_LOGI(TAG, "Starting OTA Web Server on port %d", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t update_uri = {
            .uri       = "/update",
            .method    = HTTP_POST,
            .handler   = update_post_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &update_uri);
        return ESP_OK;
    }

    return ESP_FAIL;
}
