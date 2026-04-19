#include "hal/imu_module.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

static const char *TAG = "imu_module";

#define IMU_RESET_LOW_MS 15
#define IMU_PROBE_READ_RETRY_MS 20
#define IMU_REPORT_INTERVAL_US 20000

static bool bno_int_asserted(const imu_module_t *imu)
{
    int level = gpio_get_level(imu->int_io);
    return imu->int_active_low ? (level == 0) : (level == 1);
}

static imu_module_t *imu_from_hal(sh2_Hal_t *self)
{
    return (imu_module_t *)self;
}

static void imu_cleanup_transport(imu_module_t *imu)
{
    if (imu->spi_dev != NULL) {
        (void)spi_bus_remove_device((spi_device_handle_t)imu->spi_dev);
        imu->spi_dev = NULL;
    }

    esp_err_t free_err = spi_bus_free((spi_host_device_t)imu->spi_host);
    if (free_err != ESP_OK && free_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "spi_bus_free failed: %s", esp_err_to_name(free_err));
    }
}

static esp_err_t imu_prepare_transport(imu_module_t *imu)
{
    imu_cleanup_transport(imu);

    gpio_config_t int_cfg = {
        .pin_bit_mask = (1ULL << imu->int_io),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&int_cfg);
    if (err != ESP_OK) {
        return err;
    }

    gpio_config_t rst_cfg = {
        .pin_bit_mask = (1ULL << imu->rst_io),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&rst_cfg);
    if (err != ESP_OK) {
        return err;
    }

    if (imu->wake_io >= 0) {
        gpio_config_t wake_cfg = {
            .pin_bit_mask = (1ULL << imu->wake_io),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        err = gpio_config(&wake_cfg);
        if (err != ESP_OK) {
            return err;
        }
    }

    gpio_set_level(imu->rst_io, 1);

    return ESP_OK;
}

// Mirrors Adafruit's hal_hardwareReset() — called BEFORE sh2_open()
// Follows BNO08X datasheet SPI startup sequence:
//   1. NRST low (min 10ns)
//   2. NRST high
//   3. Wait t1+t2 = ~94ms for internal init
//   4. Poll H_INTN low (device ready)
static void imu_hardware_reset(imu_module_t *imu)
{
    // Clear any previous peripheral assignments (like JTAG on 41/42)
    gpio_reset_pin(imu->mosi_io);
    gpio_reset_pin(imu->cs_io);
    gpio_reset_pin(imu->rst_io);
    if (imu->wake_io >= 0) gpio_reset_pin(imu->wake_io);

    // Explicitly configure all control pins as standard GPIO outputs
    // so we can forcibly hold them HIGH during reset.
    uint64_t mask = (1ULL << imu->mosi_io) | (1ULL << imu->cs_io) | (1ULL << imu->rst_io);
    if (imu->wake_io >= 0) mask |= (1ULL << imu->wake_io);

    gpio_config_t io_conf = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_INPUT_OUTPUT, // Enable input so get_level works
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // Set initial idle states (all HIGH)
    gpio_set_level(imu->mosi_io, 1);  // BOOTN=1 (firmware mode, not bootloader)
    gpio_set_level(imu->cs_io, 1);    // CS=1 (inactive)
    gpio_set_level(imu->rst_io, 1);   // RST=1 (not in reset)
    if (imu->wake_io >= 0) {
        gpio_set_level(imu->wake_io, 1); // PS0=HIGH = SPI mode
    }

    // Diagnostic: print and hold for 1s so user can verify with multimeter
    ESP_LOGW(TAG, "--- PIN STATES (probe now with multimeter!) ---");
    ESP_LOGW(TAG, "  MOSI/BOOTN GPIO%d = %d  (must be 1 = 3.3V)",
             imu->mosi_io, gpio_get_level(imu->mosi_io));
    ESP_LOGW(TAG, "  CS         GPIO%d = %d  (must be 1 = 3.3V)",
             imu->cs_io, gpio_get_level(imu->cs_io));
    ESP_LOGW(TAG, "  PS0/WAKE   GPIO%d = %d  (must be 1 = 3.3V)",
             imu->wake_io, gpio_get_level(imu->wake_io));
    ESP_LOGW(TAG, "  INT        GPIO%d = %d  (pull-up, idle = 1)",
             imu->int_io, gpio_get_level(imu->int_io));
    ESP_LOGW(TAG, "  RST        GPIO%d = %d  (will pulse LOW next)",
             imu->rst_io, gpio_get_level(imu->rst_io));
    ESP_LOGW(TAG, "  PS1 must be tied to 3.3V physically");
    vTaskDelay(pdMS_TO_TICKS(1000)); // 1s to see the log

    // Datasheet step 1+2: Pulse NRST LOW for 10ms
    gpio_set_level(imu->rst_io, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(imu->rst_io, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(imu->rst_io, 1);

    // Datasheet step 3: Wait t1+t2 (~94ms) for BNO085 internal init
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "RST de-asserted, polling INT for 600ms...");

    // Datasheet step 4: Poll H_INTN (600ms window, 100us samples)
    bool asserted = false;
    for (int i = 0; i < 6000; i++) {
        esp_rom_delay_us(100);
        int level = gpio_get_level(imu->int_io);
        if (imu->int_active_low ? (level == 0) : (level == 1)) {
            asserted = true;
            ESP_LOGI(TAG, "INT asserted at ~%dms after RST", 100 + (i / 10));
            break;
        }
    }

    ESP_LOGI(TAG, "Hardware reset done: INT=%s, final_level=%d",
             asserted ? "ASSERTED (LOW)" : "NOT asserted (HIGH)",
             gpio_get_level(imu->int_io));
}


// HAL open: SPI bus already set up by imu_hardware_reset path.
// Just wait for INT like Adafruit's spihal_open() does.
static int imu_hal_open(sh2_Hal_t *self)
{
    imu_module_t *imu = imu_from_hal(self);

    // Wait for INT (up to 500ms), mirroring spihal_wait_for_int()
    for (int i = 0; i < 500; i++) {
        if (bno_int_asserted(imu)) {
            return 0;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    ESP_LOGW(TAG, "imu_hal_open: INT never asserted");
    return 0; // Return 0 anyway so sh2_open() can proceed
}

static void imu_hal_close(sh2_Hal_t *self)
{
    imu_module_t *imu = imu_from_hal(self);
    imu_cleanup_transport(imu);
    gpio_set_level(imu->rst_io, 0);
    if (imu->wake_io >= 0) {
        gpio_set_level(imu->wake_io, 1);
    }
}

static int imu_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us)
{
    imu_module_t *imu = imu_from_hal(self);

    if (imu->spi_dev == NULL || !bno_int_asserted(imu) || len < 4) {
        return 0;
    }

    // Assert CS LOW
    gpio_set_level(imu->cs_io, 0);

    uint8_t header[4] = {0};
    spi_transaction_t header_t = {
        .length = sizeof(header) * 8,
        .rxlength = sizeof(header) * 8,
        .rx_buffer = header,
    };

    if (spi_device_transmit((spi_device_handle_t)imu->spi_dev, &header_t) != ESP_OK) {
        gpio_set_level(imu->cs_io, 1);
        return 0;
    }

    ESP_LOGD(TAG, "Read raw header: %02X %02X %02X %02X", header[0], header[1], header[2], header[3]);

    if (header[0] == 0xFF && header[1] == 0xFF) {
        gpio_set_level(imu->cs_io, 1);
        return 0;
    }

    uint16_t packet_len = ((uint16_t)(header[1] & 0x7F) << 8) | header[0];
    if (packet_len < sizeof(header) || packet_len > len) {
        if (packet_len > len) {
            uint8_t drain_buf[64] = {0};
            size_t remaining = packet_len - sizeof(header);
            while (remaining > 0) {
                size_t chunk = remaining > sizeof(drain_buf) ? sizeof(drain_buf) : remaining;
                spi_transaction_t drain_t = {
                    .length = chunk * 8,
                    .rxlength = chunk * 8,
                    .rx_buffer = drain_buf,
                };
                if (spi_device_transmit((spi_device_handle_t)imu->spi_dev, &drain_t) != ESP_OK) {
                    break;
                }
                remaining -= chunk;
            }
        }
        gpio_set_level(imu->cs_io, 1);
        return 0;
    }

    memcpy(pBuffer, header, sizeof(header));
    if (packet_len > sizeof(header)) {
        spi_transaction_t payload_t = {
            .length = (packet_len - sizeof(header)) * 8,
            .rxlength = (packet_len - sizeof(header)) * 8,
            .rx_buffer = pBuffer + sizeof(header),
        };
        if (spi_device_transmit((spi_device_handle_t)imu->spi_dev, &payload_t) != ESP_OK) {
            gpio_set_level(imu->cs_io, 1);
            return 0;
        }
    }

    // De-assert CS HIGH
    gpio_set_level(imu->cs_io, 1);

    if (t_us != NULL) {
        *t_us = (uint32_t)esp_timer_get_time();
    }

    return packet_len;
}

static int imu_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    imu_module_t *imu = imu_from_hal(self);

    if (imu->spi_dev == NULL) {
        return -1;
    }

    if (!bno_int_asserted(imu)) {
        return 0;
    }

    // Assert CS LOW
    gpio_set_level(imu->cs_io, 0);

    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = pBuffer,
    };

    if (spi_device_transmit((spi_device_handle_t)imu->spi_dev, &t) != ESP_OK) {
        gpio_set_level(imu->cs_io, 1);
        return -1;
    }

    // De-assert CS HIGH
    gpio_set_level(imu->cs_io, 1);

    return (int)len;
}

static uint32_t imu_hal_get_time_us(sh2_Hal_t *self)
{
    (void)self;
    return (uint32_t)esp_timer_get_time();
}

static void imu_event_callback(void *cookie, sh2_AsyncEvent_t *event)
{
    imu_module_t *imu = (imu_module_t *)cookie;
    (void)imu;

    if (event->eventId == SH2_RESET) {
        ESP_LOGI(TAG, "SH2 reset complete");
        return;
    }

    if (event->eventId == SH2_SHTP_EVENT) {
        ESP_LOGW(TAG, "SHTP event: %u", (unsigned)event->shtpEvent);
        return;
    }
}

static void imu_sensor_callback(void *cookie, sh2_SensorEvent_t *event)
{
    imu_module_t *imu = (imu_module_t *)cookie;
    sh2_SensorValue_t value = {0};

    imu->isr_count++;

    if (sh2_decodeSensorEvent(&value, event) != SH2_OK) {
        ESP_LOGI(TAG, "Report id=0x%02X len=%u", event->reportId, event->len);
        return;
    }

    switch (value.sensorId) {
        case SH2_ROTATION_VECTOR:
            ESP_LOGI(TAG, "RV q=(%.4f, %.4f, %.4f, %.4f)",
                     value.un.rotationVector.i,
                     value.un.rotationVector.j,
                     value.un.rotationVector.k,
                     value.un.rotationVector.real);
            break;
        case SH2_ACCELEROMETER:
            ESP_LOGI(TAG, "ACC mps2=(%.4f, %.4f, %.4f)",
                     value.un.accelerometer.x,
                     value.un.accelerometer.y,
                     value.un.accelerometer.z);
            break;
        case SH2_GYROSCOPE_CALIBRATED:
            ESP_LOGI(TAG, "GYR radps=(%.4f, %.4f, %.4f)",
                     value.un.gyroscope.x,
                     value.un.gyroscope.y,
                     value.un.gyroscope.z);
            break;
        default:
            ESP_LOGI(TAG, "Sensor id=0x%02X seq=%u status=%u",
                     value.sensorId,
                     value.sequence,
                     value.status);
            break;
    }
}

esp_err_t imu_module_init(imu_module_t *imu)
{
    if (imu->spi_clock_hz <= 0) {
        imu->spi_clock_hz = 3000000;
    }
    if (imu->spi_mode < 0 || imu->spi_mode > 3) {
        imu->spi_mode = 3;
    }

    imu->hal.open = imu_hal_open;
    imu->hal.close = imu_hal_close;
    imu->hal.read = imu_hal_read;
    imu->hal.write = imu_hal_write;
    imu->hal.getTimeUs = imu_hal_get_time_us;

    imu->isr_count = 0;

    // Step 1: Configure GPIO pins (INT, RST, WAKE, MOSI, CS)
    esp_err_t err = imu_prepare_transport(imu);
    if (err != ESP_OK) {
        return err;
    }

    // Step 2: Hardware reset BEFORE SPI initialization!
    //         We explicitly configure MOSI/CS as GPIOs to force them high.
    //         If we initialize ESP's SPI driver first, it steals MOSI and keeps
    //         it low, forcing the sensor into the bootloader!
    imu_hardware_reset(imu);

    // Step 3: Initialize SPI bus NOW that the BNO085 has safely booted.
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = imu->mosi_io,
        .miso_io_num = imu->miso_io,
        .sclk_io_num = imu->sclk_io,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 512,
    };
    err = spi_bus_initialize((spi_host_device_t)imu->spi_host, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        return err;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = imu->spi_clock_hz,
        .mode = imu->spi_mode,
        .spics_io_num = -1, // CS controlled manually
        .queue_size = 4,
    };
    spi_device_handle_t dev_handle = NULL;
    err = spi_bus_add_device((spi_host_device_t)imu->spi_host, &dev_cfg, &dev_handle);
    if (err != ESP_OK) {
        spi_bus_free((spi_host_device_t)imu->spi_host);
        return err;
    }
    imu->spi_dev = dev_handle;

    // Step 4: Open the SH-2 stack. This calls imu_hal_open() internally,
    //         which now only waits for INT (SPI already initialized above).
    int sh2_rc = sh2_open(&imu->hal, imu_event_callback, imu);
    if (sh2_rc != SH2_OK) {
        imu_hal_close(&imu->hal);
        return ESP_FAIL;
    }

    sh2_setSensorCallback(imu_sensor_callback, imu);

    ESP_LOGI(TAG, "BNO085 init: mode=%d SCLK=%d MOSI=%d MISO=%d CS=%d INT=%d RST=%d WAKE=%d INT_level=%d",
             imu->spi_mode,
             imu->sclk_io,
             imu->mosi_io,
             imu->miso_io,
             imu->cs_io,
             imu->int_io,
             imu->rst_io,
             imu->wake_io,
             gpio_get_level(imu->int_io));

    return ESP_OK;
}

bool imu_module_probe(imu_module_t *imu, uint32_t timeout_ms)
{
    (void)timeout_ms;

    sh2_ProductIds_t prod_ids = {0};
    int sh2_rc = sh2_getProdIds(&prod_ids);
    if (sh2_rc != SH2_OK || prod_ids.numEntries == 0) {
        ESP_LOGW(TAG, "Probe failed: no product ID response (rc=%d, entries=%u)",
                 sh2_rc,
                 (unsigned)prod_ids.numEntries);
        sh2_close();
        return false;
    }

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

void imu_module_enable_default_reports(imu_module_t *imu)
{
    sh2_SensorConfig_t cfg = {0};
    cfg.reportInterval_us = IMU_REPORT_INTERVAL_US;

    int rc = sh2_setSensorConfig(SH2_ACCELEROMETER, &cfg);
    if (rc != SH2_OK) {
        ESP_LOGE(TAG, "Failed to enable accelerometer: %d", rc);
    }

    rc = sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &cfg);
    if (rc != SH2_OK) {
        ESP_LOGE(TAG, "Failed to enable gyroscope: %d", rc);
    }

    rc = sh2_setSensorConfig(SH2_ROTATION_VECTOR, &cfg);
    if (rc != SH2_OK) {
        ESP_LOGE(TAG, "Failed to enable rotation vector: %d", rc);
    }

    ESP_LOGI(TAG, "Requested ACC/GYR/RV reports at 50Hz");
    (void)imu;
}

void imu_module_poll_and_log(imu_module_t *imu)
{
    (void)imu;
    sh2_service();
}
