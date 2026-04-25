#ifndef WIFI_MODULE_H
#define WIFI_MODULE_H

#include "esp_err.h"

/**
 * @brief Initializes the WiFi in Access Point (AP) mode.
 * 
 * @param ssid The name of the WiFi network to create.
 * @param password The password for the network.
 * @return ESP_OK on success.
 */
esp_err_t wifi_module_init_ap(const char *ssid, const char *password);

#endif
