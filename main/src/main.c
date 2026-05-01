#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "hal/imu_module.h"
#include "hal/motor_module.h"

static const char *TAG = "motor_test";

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float target_pitch;
} pid_state_t;

typedef struct {
    imu_module_t imu;
    motor_module_t motor_hal;
    motor_command_t test_cmd;
    int64_t motion_start_us;
    TaskHandle_t imu_task;
    TaskHandle_t control_task;
    TaskHandle_t supervisor_task;
    QueueHandle_t imu_queue;
    pid_state_t balance_pid;
} robot_runtime_t;

static robot_runtime_t g_runtime = {0};

// TMC2240 SPI test wiring (adjust to your board).
#define TMC_SPI_HOST        SPI2_HOST
#define TMC_SPI_SCLK        GPIO_NUM_16
#define TMC_SPI_MOSI        GPIO_NUM_15
#define TMC_SPI_MISO        GPIO_NUM_10
#define TMC_EN              GPIO_NUM_18

// ============== LEFT MOTOR PINS ==============

#define TMC_LEFT_STEP       GPIO_NUM_8
#define TMC_LEFT_DIR        GPIO_NUM_13
#define TMC_LEFT_SPI_CS     GPIO_NUM_21
#define TMC_LEFT_DIAG0      GPIO_NUM_6

// ============== RIGHT MOTOR PINS ==============
#define TMC_RIGHT_STEP      GPIO_NUM_2
#define TMC_RIGHT_DIR       GPIO_NUM_1
#define TMC_RIGHT_SPI_CS    GPIO_NUM_39
#define TMC_RIGHT_DIAG0     GPIO_NUM_41

static void imu_task_fn(void *arg)
{
    robot_runtime_t *runtime = (robot_runtime_t *)arg;
    if (runtime == NULL || runtime->imu_queue == NULL) {
        vTaskDelete(NULL);
        return;
    }

    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        if (runtime->imu.i2c_dev != NULL) {
            // Poll the IMU. In the future, this can be triggered by a hardware INT pin.
            imu_module_poll_and_log(&runtime->imu);
            
            imu_sample_t sample;
            if (imu_module_read_sample(&runtime->imu, &sample) == ESP_OK) {
                // Send the sample to the control task, overwriting if queue is full
                xQueueOverwrite(runtime->imu_queue, &sample);
            }
        }
        // Poll at 100Hz
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}

static void control_task_fn(void *arg)
{
    robot_runtime_t *runtime = (robot_runtime_t *)arg;
    if (runtime == NULL || runtime->imu_queue == NULL) {
        vTaskDelete(NULL);
        return;
    }

    motor_module_set_enabled(&runtime->motor_hal, true);
    ESP_LOGI(TAG, "Control Task Started. Waiting for IMU data...");

    // Motor and Pitch polarity variables. Flip to -1.0f if robot drives backwards or falls over.
    const float PITCH_POLARITY = 1.0f;
    const float MOTOR_POLARITY = 1.0f;

    while (1) {
        imu_sample_t sample;
        // Block indefinitely until new IMU data arrives from the IMU task
        if (xQueueReceive(runtime->imu_queue, &sample, portMAX_DELAY) == pdTRUE) {
            
            // 1. Get current pitch
            float current_pitch = sample.pitch_deg * PITCH_POLARITY;
            
            // 2. Calculate error
            float error = runtime->balance_pid.target_pitch - current_pitch;
            
            // 3. PID Math (assuming fixed dt based on IMU poll rate, ~0.01s)
            float dt = 0.01f;
            
            float p_out = runtime->balance_pid.kp * error;
            
            runtime->balance_pid.integral += error * dt;
            // Anti-windup (limit integral to +/- 1000 for now)
            if (runtime->balance_pid.integral > 1000.0f) runtime->balance_pid.integral = 1000.0f;
            if (runtime->balance_pid.integral < -1000.0f) runtime->balance_pid.integral = -1000.0f;
            float i_out = runtime->balance_pid.ki * runtime->balance_pid.integral;
            
            float derivative = (error - runtime->balance_pid.prev_error) / dt;
            float d_out = runtime->balance_pid.kd * derivative;
            
            runtime->balance_pid.prev_error = error;
            
            float total_output = p_out + i_out + d_out;
            
            // 4. Convert PID output to motor speed (Hz)
            float motor_hz = total_output * MOTOR_POLARITY;
            
            // 5. Apply to motors (Assuming both motors face same direction relative to body)
            runtime->test_cmd.left_step_hz = motor_hz;
            runtime->test_cmd.right_step_hz = motor_hz; 
            motor_module_apply_command(&runtime->motor_hal, &runtime->test_cmd);
        }
    }
}

static void supervisor_task_fn(void *arg)
{
    robot_runtime_t *runtime = (robot_runtime_t *)arg;
    if (runtime == NULL) {
        vTaskDelete(NULL);
        return;
    }

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        // Here we can eventually handle WiFi UDP, steering inputs, etc.
        // For now, just log TMC temps periodically
        motor_module_tmc2240_test_log();

        // Print stats at 1Hz
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    g_runtime.imu = (imu_module_t){
        .i2c_port = I2C_NUM_0,
        .sda_io = GPIO_NUM_5,
        .scl_io = GPIO_NUM_4,
        .i2c_clock_hz = 100000,
        .i2c_address = 0,
        .int_io = -1,
    };

    if (imu_module_init(&g_runtime.imu) != ESP_OK) {
        ESP_LOGW(TAG, "IMU init failed, continuing to motor test...");
    } else {
        if (imu_module_probe(&g_runtime.imu, 2000)) {
            bno08x_enable_game_rv(10000);
            bno08x_enable_gyroscope(10000);
            ESP_LOGI(TAG, "IMU Ready");
        }
    }

    if (motor_module_tmc2240_spi_init(TMC_SPI_HOST,
                                      TMC_SPI_SCLK,
                                      TMC_SPI_MOSI,
                                      TMC_SPI_MISO,
                                      TMC_LEFT_SPI_CS,
                                      1000000,
                                      false) != ESP_OK) {
        ESP_LOGE(TAG, "LEFT TMC2240 SPI init failed");
    }

    if (motor_module_tmc2240_spi_init(TMC_SPI_HOST,
                                      TMC_SPI_SCLK,
                                      TMC_SPI_MOSI,
                                      TMC_SPI_MISO,
                                      TMC_RIGHT_SPI_CS,
                                      1000000,
                                      true) != ESP_OK) {
        ESP_LOGE(TAG, "RIGHT TMC2240 SPI init failed");
    }

    TMC2240_RobotConfig_t silent_config = {
        .run_current_ma = 800,
        .hold_current_ma = 200,
        .microsteps = 16,
        .interpolate = true,
        .stealth_threshold = 500,
        .stall_sensitivity = 0,
        .cool_step_enabled = false,
    };

    motor_module_tmc2240_configure_robot_mode(&silent_config);

    g_runtime.motor_hal.left.step_pin = TMC_LEFT_STEP;
    g_runtime.motor_hal.left.dir_pin = TMC_LEFT_DIR;
    g_runtime.motor_hal.left.en_pin = TMC_EN;
    g_runtime.motor_hal.left.en_active_low = true;
    g_runtime.motor_hal.right.step_pin = TMC_RIGHT_STEP;
    g_runtime.motor_hal.right.dir_pin = TMC_RIGHT_DIR;
    g_runtime.motor_hal.right.en_pin = TMC_EN;
    g_runtime.motor_hal.right.en_active_low = true;
    g_runtime.motor_hal.max_step_hz = 2000.0f;
    motor_module_init(&g_runtime.motor_hal);
    motor_module_set_enabled(&g_runtime.motor_hal, false);

    // Initialize IMU Queue (Length 1, so we always only process the newest data)
    g_runtime.imu_queue = xQueueCreate(1, sizeof(imu_sample_t));

    // Initialize PID config
    g_runtime.balance_pid.kp = 20.0f;  // Tuning placeholders
    g_runtime.balance_pid.ki = 0.0f;
    g_runtime.balance_pid.kd = 0.0f;
    g_runtime.balance_pid.integral = 0.0f;
    g_runtime.balance_pid.prev_error = 0.0f;
    g_runtime.balance_pid.target_pitch = 0.0f; // Stand perfectly upright

    if (xTaskCreatePinnedToCore(imu_task_fn,
                                "imu_core0",
                                4096,
                                &g_runtime,
                                4,
                                &g_runtime.imu_task,
                                0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create IMU task on Core 0");
        return;
    }

    if (xTaskCreatePinnedToCore(control_task_fn,
                                "control_core1",
                                4096,
                                &g_runtime,
                                5, // Highest priority
                                &g_runtime.control_task,
                                1) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create control task on Core 1");
        return;
    }

    if (xTaskCreatePinnedToCore(supervisor_task_fn,
                                "supervisor_core0",
                                4096,
                                &g_runtime,
                                2, // Low priority
                                &g_runtime.supervisor_task,
                                0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create supervisor task on Core 0");
        return;
    }

    vTaskDelete(NULL);
}
