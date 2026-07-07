#ifndef IMU_MODULE_H
#define IMU_MODULE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "sh2_SensorValue.h"
#include "sh2_hal.h"
#include "robot_types.h"

#define BNO08X_MAX_PACKET_LEN 512

typedef struct {
    sh2_Hal_t hal;

    i2c_master_bus_handle_t i2c_bus;
    i2c_master_dev_handle_t i2c_dev;
    int i2c_port;
    int sda_io;
    int scl_io;
    int i2c_clock_hz;
    uint8_t i2c_address;
    volatile bool reset_seen;
    volatile bool have_sample;
    sh2_SensorValue_t sensor_value;
    int64_t last_print_us;
    uint8_t last_packet[BNO08X_MAX_PACKET_LEN];
    uint16_t last_packet_len;
    volatile bool packet_ready;
    float yaw_deg;
    float pitch_deg;
    float roll_deg;
    float gyro_pitch_dps;
    float lin_accel_x;
    bool orientation_valid;
    bool gyro_valid;
    bool lin_accel_valid;
    uint8_t control_seq;
    float tilt_zero_deg;

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
    void *task_to_notify;
} imu_module_t;

esp_err_t imu_module_init(imu_module_t *imu);
bool imu_module_probe(imu_module_t *imu, uint32_t timeout_ms);
esp_err_t bno08x_enable_game_rv(uint32_t interval_us);
esp_err_t bno08x_enable_gyroscope(uint32_t interval_us);
esp_err_t bno08x_enable_accelerometer(uint32_t interval_us);
esp_err_t bno08x_enable_all_reports(uint32_t interval_us);
void imu_module_enable_default_reports(imu_module_t *imu);
void imu_module_poll_and_log(imu_module_t *imu);

esp_err_t imu_module_read_sample(imu_module_t *imu, imu_sample_t *sample);
void imu_module_set_notify_task(imu_module_t *imu, void *task_handle);

#endif