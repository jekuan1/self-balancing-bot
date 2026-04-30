#include "hal/imu_module.h"

#include <math.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

static const char *TAG = "imu_module";
static imu_module_t *s_active_imu = NULL;

#define IMU_I2C_TIMEOUT_MS 1000
#define IMU_PROBE_READ_RETRY_MS 20
#define IMU_GAME_RV_INTERVAL_US 5000
#define BNO08X_SENSOR_ADDR 0x4A
#define BNO08X_ALT_SENSOR_ADDR 0x4B
#define BNO08X_SOFT_RESET_LEN 5

static float quat_to_yaw_deg(float real, float i, float j, float k)
{
    // Yaw (Z-axis rotation)
    float yaw = atan2f(2.0f * (real * k + i * j), 1.0f - 2.0f * (j * j + k * k));
    return yaw * 57.2957795f;
}

static float quat_to_pitch_deg(float real, float i, float j, float k)
{
    // Pitch (Y-axis rotation)
    float sinp = 2.0f * (real * j - i * k);
    float pitch;
    if (fabsf(sinp) >= 1.0f) {
        pitch = copysignf(M_PI / 2.0f, sinp); // Use 90 degrees if out of range
    } else {
        pitch = asinf(sinp);
    }
    return pitch * 57.2957795f;
}

static float quat_to_roll_deg(float real, float i, float j, float k)
{
    // Roll (X-axis rotation)
    float roll = atan2f(2.0f * (real * i + j * k), 1.0f - 2.0f * (i * i + j * j));
    return roll * 57.2957795f;
}


static imu_module_t *imu_from_hal(sh2_Hal_t *self)
{
    return (imu_module_t *)self;
}

static void imu_reset_report_state(imu_module_t *imu)
{
    imu->yaw_deg = 0.0f;
    imu->pitch_deg = 0.0f;
    imu->roll_deg = 0.0f;
    imu->gyro_pitch_dps = 0.0f;
    imu->orientation_valid = false;
    imu->gyro_valid = false;
    imu->control_seq = 0;
    memset((void *)&imu->sensor_value, 0, sizeof(imu->sensor_value));
}

static esp_err_t imu_cleanup_i2c(imu_module_t *imu)
{
    if (imu->i2c_dev != NULL) {
        (void)i2c_master_bus_rm_device(imu->i2c_dev);
        imu->i2c_dev = NULL;
    }

    if (imu->i2c_bus != NULL) {
        esp_err_t err = i2c_del_master_bus(imu->i2c_bus);
        imu->i2c_bus = NULL;
        return err;
    }

    return ESP_OK;
}

static esp_err_t imu_i2c_bus_init(imu_module_t *imu)
{
    if (imu->i2c_port < 0) {
        imu->i2c_port = I2C_NUM_0;
    }

    if (imu->i2c_clock_hz <= 0) {
        imu->i2c_clock_hz = 100000;
    }

    i2c_master_bus_config_t bus_config = {
        .i2c_port = imu->i2c_port,
        .sda_io_num = imu->sda_io,
        .scl_io_num = imu->scl_io,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };

    return i2c_new_master_bus(&bus_config, &imu->i2c_bus);
}

static esp_err_t imu_i2c_add_device(imu_module_t *imu,
                                    i2c_master_dev_handle_t *dev_handle,
                                    uint8_t device_address)
{
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_address,
        .scl_speed_hz = imu->i2c_clock_hz,
    };

    return i2c_master_bus_add_device(imu->i2c_bus, &dev_config, dev_handle);
}

static bool imu_i2c_address_responds(imu_module_t *imu, uint8_t addr)
{
    i2c_master_dev_handle_t temp_handle = NULL;
    uint8_t dummy = 0;

    if (imu_i2c_add_device(imu, &temp_handle, addr) != ESP_OK) {
        return false;
    }

    esp_err_t ret = i2c_master_transmit(temp_handle, &dummy, 1, IMU_I2C_TIMEOUT_MS);
    (void)i2c_master_bus_rm_device(temp_handle);
    return ret == ESP_OK;
}

static esp_err_t imu_send_soft_reset(imu_module_t *imu)
{
    static const uint8_t reset_pkt[BNO08X_SOFT_RESET_LEN] = {0x05, 0x00, 0x01, 0x00, 0x01};
    return i2c_master_transmit(imu->i2c_dev, reset_pkt, sizeof(reset_pkt), IMU_I2C_TIMEOUT_MS);
}

static esp_err_t imu_enable_report(sh2_SensorId_t sensor_id, uint32_t interval_us)
{
    sh2_SensorConfig_t cfg = {
        .changeSensitivityEnabled = false,
        .changeSensitivityRelative = false,
        .wakeupEnabled = false,
        .alwaysOnEnabled = false,
        .changeSensitivity = 0,
        .reportInterval_us = interval_us,
        .batchInterval_us = 0,
        .sensorSpecific = 0,
    };

    return (sh2_setSensorConfig(sensor_id, &cfg) == SH2_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t bno08x_enable_game_rv(uint32_t interval_us)
{
    imu_module_t *imu = s_active_imu;
    if (imu == NULL) {
        return ESP_FAIL;
    }

    if (imu_enable_report(SH2_GAME_ROTATION_VECTOR, interval_us) != ESP_OK) {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Requested Game Rotation Vector at %u us", (unsigned)interval_us);
    return ESP_OK;
}

esp_err_t bno08x_enable_gyroscope(uint32_t interval_us)
{
    sh2_SensorConfig_t config;
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.reportInterval_us = interval_us;
    config.batchInterval_us = 0;

    int rc = sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &config);
    return (rc == SH2_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t bno08x_enable_accelerometer(uint32_t interval_us)
{
    imu_module_t *imu = s_active_imu;
    if (imu == NULL) {
        return ESP_FAIL;
    }

    if (imu_enable_report(SH2_ACCELEROMETER, interval_us) != ESP_OK) {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Requested Accelerometer at %u us", (unsigned)interval_us);
    return ESP_OK;
}

esp_err_t bno08x_enable_all_reports(uint32_t interval_us)
{
    const sh2_SensorId_t sensors[] = {
        SH2_ACCELEROMETER,
        SH2_GYROSCOPE_CALIBRATED,
        SH2_MAGNETIC_FIELD_CALIBRATED,
        SH2_LINEAR_ACCELERATION,
        SH2_GRAVITY,
        SH2_ROTATION_VECTOR,
        SH2_GAME_ROTATION_VECTOR,
    };

    for (size_t i = 0; i < (sizeof(sensors) / sizeof(sensors[0])); i++) {
        if (imu_enable_report(sensors[i], interval_us) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable sensor id=0x%02X", sensors[i]);
            return ESP_FAIL;
        }
    }

    ESP_LOGI(TAG, "Requested all primary IMU reports at %u us", (unsigned)interval_us);
    return ESP_OK;
}

static int imu_hal_open(sh2_Hal_t *self)
{
    imu_module_t *imu = imu_from_hal(self);

    if (imu_send_soft_reset(imu) != ESP_OK) {
        return SH2_ERR;
    }

    vTaskDelay(pdMS_TO_TICKS(300));
    return SH2_OK;
}

static void imu_hal_close(sh2_Hal_t *self)
{
    imu_module_t *imu = imu_from_hal(self);
    (void)imu_cleanup_i2c(imu);
}

static int imu_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us)
{
    imu_module_t *imu = imu_from_hal(self);

    if (imu->i2c_dev == NULL || len < 4U) {
        return 0;
    }

    uint8_t header[4] = {0};
    if (i2c_master_receive(imu->i2c_dev, header, sizeof(header), IMU_I2C_TIMEOUT_MS) != ESP_OK) {
        return 0;
    }

    if (header[0] == 0xFF && header[1] == 0xFF) {
        return 0;
    }

    uint16_t packet_len = (uint16_t)header[0] | ((uint16_t)header[1] << 8);
    packet_len &= (uint16_t)~0x8000U;

    if (packet_len < 4U || packet_len > len) {
        return 0;
    }

    // Match the known-good BNO08x I2C SH-2 read flow.
    if (i2c_master_receive(imu->i2c_dev, pBuffer, packet_len, IMU_I2C_TIMEOUT_MS) != ESP_OK) {
        return 0;
    }

    if (t_us != NULL) {
        *t_us = (uint32_t)esp_timer_get_time();
    }

    return (int)packet_len;
}

static int imu_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    imu_module_t *imu = imu_from_hal(self);

    if (imu->i2c_dev == NULL) {
        return 0;
    }

    return (i2c_master_transmit(imu->i2c_dev, pBuffer, len, IMU_I2C_TIMEOUT_MS) == ESP_OK) ? (int)len : 0;
}

static uint32_t imu_hal_get_time_us(sh2_Hal_t *self)
{
    (void)self;
    return (uint32_t)esp_timer_get_time();
}

static void imu_event_callback(void *cookie, sh2_AsyncEvent_t *event)
{
    imu_module_t *imu = (imu_module_t *)cookie;

    if (event->eventId == SH2_RESET) {
        imu->reset_seen = true;
        ESP_LOGI(TAG, "SH-2 reset complete");
        return;
    }

    if (event->eventId == SH2_SHTP_EVENT) {
        ESP_LOGW(TAG, "SHTP event: %u", (unsigned)event->shtpEvent);
    }
}

static void imu_sensor_callback(void *cookie, sh2_SensorEvent_t *event)
{
    imu_module_t *imu = (imu_module_t *)cookie;
    sh2_SensorValue_t temp = {0};

    imu->isr_count++;

    if (sh2_decodeSensorEvent(&temp, event) != SH2_OK) {
        return;
    }

    if (temp.sensorId == SH2_GAME_ROTATION_VECTOR) {
        imu->yaw_deg = quat_to_yaw_deg(temp.un.gameRotationVector.real,
                                      temp.un.gameRotationVector.i,
                                      temp.un.gameRotationVector.j,
                                      temp.un.gameRotationVector.k);
        imu->pitch_deg = quat_to_pitch_deg(temp.un.gameRotationVector.real,
                                          temp.un.gameRotationVector.i,
                                          temp.un.gameRotationVector.j,
                                          temp.un.gameRotationVector.k);
        imu->roll_deg = quat_to_roll_deg(temp.un.gameRotationVector.real,
                                        temp.un.gameRotationVector.i,
                                        temp.un.gameRotationVector.j,
                                        temp.un.gameRotationVector.k);
        imu->orientation_valid = true;
    } else if (temp.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        // Since the IMU is mounted vertically for balancing, 
        // the gyro axis we care about depends on mounting.
        // Assuming pitch is around X or Y. 
        imu->gyro_pitch_dps = temp.un.gyroscope.y * 57.2957795f; 
        imu->gyro_valid = true;
    }

    imu->have_sample = true;

    if (imu->task_to_notify != NULL) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        vTaskNotifyGiveFromISR((TaskHandle_t)imu->task_to_notify, &higher_priority_task_woken);
        if (higher_priority_task_woken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

static void imu_log_sample(imu_module_t *imu)
{
    if (!imu->have_sample) {
        return;
    }

    imu->have_sample = false;

    /*
    int64_t now_us = esp_timer_get_time();
    int64_t dt_us = (imu->last_print_us == 0) ? 0 : (now_us - imu->last_print_us);
    imu->last_print_us = now_us;

    if (imu->sensor_value.sensorId != SH2_ACCELEROMETER) {
        return;
    }

    ESP_LOGI(TAG, "%lld\tAX=%8.3f\tAY=%8.3f\tAZ=%8.3f",
             (long long)dt_us,
             imu->sensor_value.un.accelerometer.x,
             imu->sensor_value.un.accelerometer.y,
             imu->sensor_value.un.accelerometer.z);
    */
}

esp_err_t imu_module_init(imu_module_t *imu)
{
    imu_reset_report_state(imu);
    s_active_imu = imu;

    imu->hal.open = imu_hal_open;
    imu->hal.close = imu_hal_close;
    imu->hal.read = imu_hal_read;
    imu->hal.write = imu_hal_write;
    imu->hal.getTimeUs = imu_hal_get_time_us;

    imu->isr_count = 0;

    esp_err_t err = imu_i2c_bus_init(imu);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t active_addr = imu->i2c_address;
    if (active_addr == 0) {
        if (imu_i2c_address_responds(imu, BNO08X_SENSOR_ADDR)) {
            active_addr = BNO08X_SENSOR_ADDR;
        } else if (imu_i2c_address_responds(imu, BNO08X_ALT_SENSOR_ADDR)) {
            active_addr = BNO08X_ALT_SENSOR_ADDR;
        } else {
            ESP_LOGE(TAG, "BNO08x did not ACK at 0x%02X or 0x%02X", BNO08X_SENSOR_ADDR, BNO08X_ALT_SENSOR_ADDR);
            (void)imu_cleanup_i2c(imu);
            return ESP_FAIL;
        }
    } else if (!imu_i2c_address_responds(imu, active_addr)) {
        ESP_LOGE(TAG, "BNO08x did not ACK at 0x%02X", active_addr);
        (void)imu_cleanup_i2c(imu);
        return ESP_FAIL;
    }

    err = imu_i2c_add_device(imu, &imu->i2c_dev, active_addr);
    if (err != ESP_OK) {
        (void)imu_cleanup_i2c(imu);
        return err;
    }

    int sh2_rc = sh2_open(&imu->hal, imu_event_callback, imu);
    if (sh2_rc != SH2_OK) {
        (void)imu_cleanup_i2c(imu);
        return ESP_FAIL;
    }

    sh2_setSensorCallback(imu_sensor_callback, imu);

    ESP_LOGI(TAG, "BNO08x detected at 0x%02X (SDA=%d SCL=%d)",
             active_addr,
             imu->sda_io,
             imu->scl_io);

    return ESP_OK;
}

bool imu_module_probe(imu_module_t *imu, uint32_t timeout_ms)
{
    int64_t deadline_us = esp_timer_get_time() + ((int64_t)timeout_ms * 1000LL);
    sh2_ProductIds_t prod_ids = {0};

    while (esp_timer_get_time() <= deadline_us) {
        sh2_service();

        int sh2_rc = sh2_getProdIds(&prod_ids);
        if (sh2_rc == SH2_OK && prod_ids.numEntries > 0) {
            sh2_ProductId_t *prod = &prod_ids.entry[0];
            ESP_LOGI(TAG, "Probe OK: resetCause=%u sw=%u.%u.%u build=%u part=%u entries=%u",
                     prod->resetCause,
                     prod->swVersionMajor,
                     prod->swVersionMinor,
                     prod->swVersionPatch,
                     (unsigned)prod->swBuildNumber,
                     (unsigned)prod->swPartNumber,
                     (unsigned)prod_ids.numEntries);
            return true;
        }

        vTaskDelay(pdMS_TO_TICKS(IMU_PROBE_READ_RETRY_MS));
    }

    ESP_LOGW(TAG, "Probe failed: no product ID response within %u ms", (unsigned)timeout_ms);
    sh2_close();
    return false;
}

void imu_module_enable_default_reports(imu_module_t *imu)
{
    (void)imu;
    // Enable Game Rotation Vector (Yaw/Pitch/Roll) + Calibrated Gyro
    // 10ms (100Hz) is better for balancing if I2C holds up. 
    // Starting with 20ms (50Hz) for stability.
    uint32_t interval = 20000;
    (void)bno08x_enable_game_rv(interval);
    (void)bno08x_enable_gyroscope(interval);
}

void imu_module_poll_and_log(imu_module_t *imu)
{
    if (imu == NULL || imu->i2c_dev == NULL) {
        return;
    }

    sh2_service();

    if (imu->reset_seen) {
        imu->reset_seen = false;
        imu_module_enable_default_reports(imu);
        ESP_LOGW(TAG, "Sensor reset detected, report re-enabled");
    }

    imu_log_sample(imu);
}

void imu_module_set_notify_task(imu_module_t *imu, void *task_handle)
{
    imu->task_to_notify = task_handle;
}

esp_err_t imu_module_read_sample(imu_module_t *imu, imu_sample_t *sample)
{
    if (imu == NULL || sample == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    sample->yaw_deg = imu->yaw_deg;
    sample->pitch_deg = imu->pitch_deg - imu->tilt_zero_deg;
    sample->roll_deg = imu->roll_deg;
    sample->gyro_pitch_dps = imu->gyro_pitch_dps;
    sample->timestamp_us = esp_timer_get_time();

    return imu->orientation_valid ? ESP_OK : ESP_ERR_INVALID_STATE;
}

