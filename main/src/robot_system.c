#include "robot_system.h"

#include <math.h>
#include <string.h>

#include "control/pid_controller.h"
#include "control/state_estimator.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/imu_module.h"
#include "hal/motor_module.h"
#include "hal/safety_watchdog.h"
#include "robot_types.h"
#include "supervisor/command_parser.h"
#include "supervisor/parameter_manager.h"
#include "supervisor/telemetry_comms.h"

static const char *TAG = "robot_system";

typedef struct {
    imu_module_t imu;
    motor_module_t motor;
    safety_watchdog_t watchdog;
    state_estimator_t estimator;
    pid_controller_t pid;
    telemetry_comms_t telemetry;
    command_parser_t parser;

    robot_pose_t pose;
    control_setpoint_t setpoint;
    robot_state_t state;

    float ready_tilt_limit_deg;
    float failure_tilt_limit_deg;

    TaskHandle_t control_task;
    TaskHandle_t supervisor_task;
    portMUX_TYPE lock;
} robot_system_ctx_t;

static robot_system_ctx_t g_robot;

static void robot_emergency_stop(robot_system_ctx_t *sys, const char *reason)
{
    motor_module_set_enabled(&sys->motor, false);
    pid_controller_reset(&sys->pid);
    taskENTER_CRITICAL(&sys->lock);
    sys->state = ROBOT_STATE_FAILURE;
    taskEXIT_CRITICAL(&sys->lock);
    ESP_LOGE(TAG, "Emergency stop: %s", reason);
}

static void control_task_fn(void *arg)
{
    robot_system_ctx_t *sys = (robot_system_ctx_t *)arg;
    imu_sample_t sample;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));

        // Process incoming IMU data (Service BNO08x without logging)
        sh2_service();

        int64_t now_us = esp_timer_get_time();
        if (imu_module_read_sample(&sys->imu, &sample) == ESP_OK) {
            state_estimator_update(&sys->estimator, &sample, &sys->pose);
        }

        if (safety_watchdog_is_tripped(&sys->watchdog)) {
            robot_emergency_stop(sys, "driver DIAG asserted");
        }

        /* Temporary Disability for Testing
        if (fabsf(sys->pose.tilt_deg) > sys->failure_tilt_limit_deg) {
            robot_emergency_stop(sys, "tilt exceeded failure threshold");
        }
        */

        // Temporary Open-Loop Motor Test (Ignore PID)
        motor_module_set_enabled(&sys->motor, true);
        motor_command_t cmd = {
            .left_step_hz = 500.0f,
            .right_step_hz = 500.0f,
        };
        motor_module_apply_command(&sys->motor, &cmd);

        motor_module_service_step_pulses(&sys->motor, now_us);
    }
}

static void supervisor_task_fn(void *arg)
{
    robot_system_ctx_t *sys = (robot_system_ctx_t *)arg;

    for (;;) {
        control_setpoint_t new_setpoint;
        if (command_parser_poll(&sys->parser, &new_setpoint)) {
            taskENTER_CRITICAL(&sys->lock);
            sys->setpoint = new_setpoint;
            taskEXIT_CRITICAL(&sys->lock);
        }

        robot_pose_t pose_copy;
        robot_state_t state_copy;
        taskENTER_CRITICAL(&sys->lock);
        pose_copy = sys->pose;
        state_copy = sys->state;
        taskEXIT_CRITICAL(&sys->lock);

        telemetry_comms_publish(&sys->telemetry, state_copy, &pose_copy);

        // Periodically log motor SPI status for debugging
        motor_module_tmc2240_test_log();

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void robot_system_start(void)
{
    memset(&g_robot, 0, sizeof(g_robot));
    g_robot.lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;

    g_robot.ready_tilt_limit_deg = 5.0f;
    g_robot.failure_tilt_limit_deg = 60.0f;
    g_robot.state = ROBOT_STATE_LOCKED;

    // 1. IMU Configuration (BNO085)
    g_robot.imu.sda_io = 5;
    g_robot.imu.scl_io = 4;
    g_robot.imu.i2c_port = I2C_NUM_0;
    g_robot.imu.i2c_clock_hz = 100000;
    g_robot.imu.i2c_address = 0x4B; 
    g_robot.imu.int_io = -1; // Interrupt not used in current poll-driven mode
    g_robot.imu.tilt_zero_deg = 0.0f;

    // 2. Motor Configuration (TMC2240)
    // Left Motor Pins
    g_robot.motor.left.step_pin = 8;
    g_robot.motor.left.dir_pin = 11;
    g_robot.motor.left.en_pin = 17;
    g_robot.motor.left.en_active_low = true;

    // Right Motor Pins
    g_robot.motor.right.step_pin = 20; 
    g_robot.motor.right.dir_pin = 19; 
    g_robot.motor.right.en_pin = 17; 
    g_robot.motor.right.en_active_low = true;

    g_robot.motor.max_step_hz = 5000.0f;

    // 3. Watchdog (DIAG) Configuration
    g_robot.watchdog.diag_left_io = 6; 
    g_robot.watchdog.diag_right_io = 41;

    parameter_manager_init();

    pid_params_t params;
    if (!parameter_manager_load_pid(&params)) {
        params.kp = 45.0f;
        params.ki = 0.0f;
        params.kd = 1.2f;
    }

    pid_controller_init(&g_robot.pid, params.kp, params.ki, params.kd, -2000.0f, 2000.0f);
    state_estimator_init(&g_robot.estimator);
    command_parser_init(&g_robot.parser);
    telemetry_comms_init(&g_robot.telemetry);

    // 4. Initialize Hardware Modules
    imu_module_init(&g_robot.imu);
    imu_module_enable_default_reports(&g_robot.imu);
    
    motor_module_init(&g_robot.motor);
    
    // ISOLATION TEST: Only initializing the LEFT motor (CS=9)
    // Right motor (CS=39) is kept HIGH to deselect it
    gpio_set_level(39, 1); 
    motor_module_tmc2240_spi_init(1, 16, 15, 10, 9, -1, 50000);
    motor_module_tmc2240_configure_robot_mode();
    motor_module_tmc2240_test_log();

    // Force EN pin low (Active)
    gpio_set_level(17, 0);

    safety_watchdog_init(&g_robot.watchdog);

    g_robot.setpoint.drive_velocity = 0.0f;
    g_robot.setpoint.turn_rate = 0.0f;
    g_robot.setpoint.tilt_setpoint_deg = 0.0f;

    xTaskCreatePinnedToCore(control_task_fn, "control_core1", 4096, &g_robot, 20, &g_robot.control_task, 1);
    imu_module_set_notify_task(&g_robot.imu, g_robot.control_task);

    xTaskCreatePinnedToCore(supervisor_task_fn, "supervisor_core0", 4096, &g_robot, 6, &g_robot.supervisor_task, 0);

    ESP_LOGI(TAG, "Robot system started in LOCKED state");
}
