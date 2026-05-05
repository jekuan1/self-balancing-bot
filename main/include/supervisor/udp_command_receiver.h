#ifndef UDP_COMMAND_RECEIVER_H
#define UDP_COMMAND_RECEIVER_H

#include "esp_err.h"

/**
 * Initialize UDP command receiver on port 5555.
 * Received commands are dispatched the same way as serial commands.
 * Runs in a background task.
 */
esp_err_t udp_command_receiver_init(void);

#endif
