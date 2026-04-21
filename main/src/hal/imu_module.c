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

#define IMU_I2C_TIMEOUT_MS 1000
#define IMU_PROBE_READ_RETRY_MS 20
#define IMU_REPORT_INTERVAL_US 5000
#define BNO08X_SENSOR_ADDR 0x4A
#define BNO08X_ALT_SENSOR_ADDR 0x4B
#define BNO08X_SOFT_RESET_LEN 5

static imu_module_t *imu_from_hal(sh2_Hal_t *self)
{
    return (imu_module_t *)self;
}

static void imu_reset_report_state(imu_module_t *imu)
{
    imu->reset_seen = false;
    imu->have_sample = false;
    imu->last_print_us = 0;
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
        .flags.enable_internal_pullup = true,
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

static void quaternion_to_euler(float real, float i, float j, float k,
                                float *yaw_deg, float *pitch_deg, float *roll_deg)
{
    float sqr = real * real;
    float sqi = i * i;
    float sqj = j * j;
    float sqk = k * k;

    float yaw = atan2f(2.0f * (i * j + k * real), (sqi - sqj - sqk + sqr));
    float pitch = asinf(-2.0f * (i * k - j * real) / (sqi + sqj + sqk + sqr));
    float roll = atan2f(2.0f * (j * k + i * real), (-sqi - sqj + sqk + sqr));

    const float rad_to_deg = 57.2957795f;
    *yaw_deg = yaw * rad_to_deg;
    *pitch_deg = pitch * rad_to_deg;
    *roll_deg = roll * rad_to_deg;
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
        ESP_LOGI(TAG, "Report id=0x%02X len=%u", event->reportId, event->len);
        return;
    }

    imu->sensor_value = temp;
    imu->have_sample = true;
}

static void imu_log_sample(imu_module_t *imu)
{
    if (!imu->have_sample || imu->sensor_value.sensorId != SH2_ARVR_STABILIZED_RV) {
        return;
    }

    imu->have_sample = false;

    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
    quaternion_to_euler(
        imu->sensor_value.un.arvrStabilizedRV.real,
        imu->sensor_value.un.arvrStabilizedRV.i,
        imu->sensor_value.un.arvrStabilizedRV.j,
        imu->sensor_value.un.arvrStabilizedRV.k,
        &yaw, &pitch, &roll);

    int64_t now_us = esp_timer_get_time();
    int64_t dt_us = (imu->last_print_us == 0) ? 0 : (now_us - imu->last_print_us);
    imu->last_print_us = now_us;

    ESP_LOGI(TAG, "%lld\tacc=%u\tyaw=%.2f\tpitch=%.2f\troll=%.2f",
             (long long)dt_us,
             (unsigned)imu->sensor_value.status,
             yaw, pitch, roll);
}

esp_err_t imu_module_init(imu_module_t *imu)
{
    imu_reset_report_state(imu);

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
    sh2_SensorConfig_t cfg = {0};
    cfg.changeSensitivityEnabled = false;
    cfg.changeSensitivityRelative = false;
    cfg.wakeupEnabled = false;
    cfg.alwaysOnEnabled = false;
    cfg.changeSensitivity = 0;
    cfg.reportInterval_us = IMU_REPORT_INTERVAL_US;
    cfg.batchInterval_us = 0;
    cfg.sensorSpecific = 0;

    int rc = sh2_setSensorConfig(SH2_ARVR_STABILIZED_RV, &cfg);
    if (rc != SH2_OK) {
        ESP_LOGE(TAG, "Failed to enable SH2_ARVR_STABILIZED_RV: %d", rc);
        return;
    }

    ESP_LOGI(TAG, "Requested ARVR stabilized rotation vector at 200Hz");
    (void)imu;
}

void imu_module_poll_and_log(imu_module_t *imu)
{
    sh2_service();

    if (imu->reset_seen) {
        imu->reset_seen = false;
        imu_module_enable_default_reports(imu);
        ESP_LOGW(TAG, "Sensor reset detected, report re-enabled");
    }

    imu_log_sample(imu);
}
