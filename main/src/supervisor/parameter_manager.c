#include "supervisor/parameter_manager.h"

#include "nvs_flash.h"

void parameter_manager_init(void)
{
    (void)nvs_flash_init();
}

bool parameter_manager_load_pid(pid_params_t *out_params)
{
    out_params->kp = 45.0f;
    out_params->ki = 0.0f;
    out_params->kd = 1.2f;
    return true;
}

bool parameter_manager_save_pid(const pid_params_t *params)
{
    (void)params;
    return true;
}
