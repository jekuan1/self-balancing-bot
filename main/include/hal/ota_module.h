#ifndef OTA_MODULE_H
#define OTA_MODULE_H

#include "esp_err.h"

/**
 * @brief Initialize the OTA Web Server
 * 
 * Starts an HTTP server on port 80. 
 * Endpoint POST /update accepts a .bin file for wireless flashing.
 */
esp_err_t ota_module_init(void);

#endif
