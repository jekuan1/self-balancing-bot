#ifndef PARAMETER_MANAGER_H
#define PARAMETER_MANAGER_H

#include <stdbool.h>

typedef struct {
    float kp;
    float ki;
    float kd;
} pid_params_t;

void parameter_manager_init(void);
bool parameter_manager_load_pid(pid_params_t *out_params);
bool parameter_manager_save_pid(const pid_params_t *params);

#endif