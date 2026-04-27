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

void motor_module_init(motor_module_t *motor);
void motor_module_set_enabled(motor_module_t *motor, bool enable);
void motor_module_apply_command(motor_module_t *motor, const motor_command_t *command);
void motor_module_service_step_pulses(motor_module_t *motor, int64_t now_us);

esp_err_t motor_module_tmc2240_spi_init(int spi_host,
                                        int sclk_pin,
                                        int mosi_pin,
                                        int miso_pin,
                                        int cs_left,
                                        int cs_right,
                                        int spi_clock_hz);
esp_err_t motor_module_tmc2240_write_reg(int dev_idx, uint8_t reg_addr, uint32_t value);
esp_err_t motor_module_tmc2240_read_reg(int dev_idx, uint8_t reg_addr, uint32_t *value_out);
void motor_module_tmc2240_test_log(void);
void motor_module_tmc2240_configure_robot_mode(void);
esp_err_t motor_module_tmc2240_get_temp(int dev_idx, float *temp_c);
esp_err_t motor_module_tmc2240_get_current_scaling(int dev_idx, uint8_t *cs_actual);

#endif