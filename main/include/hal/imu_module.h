#ifndef IMU_MODULE_H
#define IMU_MODULE_H

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/task.h"
#include "robot_types.h"

typedef struct {
    spi_host_device_t spi_host;
    int mosi_io;
    int miso_io;
    int sclk_io;
    int cs_io;
    int int_io;
    float tilt_zero_deg;
    TaskHandle_t notify_task;
    volatile uint32_t isr_count;
} imu_module_t;

void imu_module_init(imu_module_t *imu);
void imu_module_set_notify_task(imu_module_t *imu, TaskHandle_t task);
bool imu_module_read_sample(imu_module_t *imu, imu_sample_t *out_sample);

#endif