#ifndef UDP_LOGGING_H
#define UDP_LOGGING_H

/**
 * @brief Initialize wireless UDP logging
 * 
 * Redirects ESP_LOG output to a remote IP/Port over WiFi.
 * 
 * @param ip_addr The laptop's IP address (usually "192.168.4.2" in AP mode)
 * @param port The UDP port to send logs to (e.g., 1234)
 */
void udp_logging_init(const char *ip_addr, int port);
void udp_logging_printf(const char *fmt, ...);

#endif
