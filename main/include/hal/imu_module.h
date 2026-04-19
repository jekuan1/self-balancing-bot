#ifndef IMU_MODULE_H
#define IMU_MODULE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "sh2_hal.h"

typedef struct {
    sh2_Hal_t hal;

    int spi_host;
    int mosi_io;
    int miso_io;
    int sclk_io;
    int cs_io;
    int int_io;
    int rst_io;
    int wake_io;
    int spi_clock_hz;
    int spi_mode;
    bool int_active_low;

    void *spi_dev;
    volatile uint32_t isr_count;
} imu_module_t;

esp_err_t imu_module_init(imu_module_t *imu);
bool imu_module_probe(imu_module_t *imu, uint32_t timeout_ms);
void imu_module_enable_default_reports(imu_module_t *imu);
void imu_module_poll_and_log(imu_module_t *imu);

#endif