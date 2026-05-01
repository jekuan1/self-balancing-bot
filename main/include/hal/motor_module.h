#ifndef MOTOR_MODULE_H
#define MOTOR_MODULE_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_err.h"
#include "robot_types.h"

typedef struct {
    int step_pin;
    int dir_pin;
    int en_pin;
    bool en_active_low;
    bool enabled;
    float target_hz;
    int64_t last_toggle_us;
    bool step_level;
} stepper_channel_t;

typedef struct {
    stepper_channel_t left;
    stepper_channel_t right;
    float max_step_hz;
} motor_module_t;

typedef struct {
    uint16_t run_current_ma;
    uint16_t hold_current_ma;
    uint16_t microsteps;
    bool interpolate;
    uint32_t stealth_threshold;
    int8_t stall_sensitivity;
    bool cool_step_enabled;
} TMC2240_RobotConfig_t;

void motor_module_init(motor_module_t *motor);
void motor_module_set_enabled(motor_module_t *motor, bool enable);
void motor_module_apply_command(motor_module_t *motor, const motor_command_t *command);
void motor_module_service_step_pulses(motor_module_t *motor, int64_t now_us);

esp_err_t motor_module_tmc2240_spi_init(int spi_host,
                                        int sclk_pin,
                                        int mosi_pin,
                                        int miso_pin,
                                        int cs_pin,
                                        int spi_clock_hz,
                                        bool right_ctx);
esp_err_t motor_module_tmc2240_write_reg(uint8_t reg_addr, uint32_t value);
esp_err_t motor_module_tmc2240_read_reg(uint8_t reg_addr, uint32_t *value_out);
void motor_module_tmc2240_test_log(void);
void motor_module_tmc2240_right_test_log(void);
void motor_module_tmc2240_debug_left(void);
void motor_module_tmc2240_debug_right(void);
void motor_module_tmc2240_configure_robot_mode(const TMC2240_RobotConfig_t *config);

#endif