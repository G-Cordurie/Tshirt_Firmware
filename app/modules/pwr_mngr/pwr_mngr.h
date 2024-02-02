#ifndef POWER_MNGR_H
#define POWER_MNGR_H

#include <stdbool.h>
#include <stdint.h>

void power_management_init(void);
void pm_allow_device_to_sleep(void);
void pm_disallow_device_to_sleep(void);
void system_shutdown(bool session_store);
void sleep_mode_enter(void);
void pwr_mngr_task(void);

#endif // POWER_MNGR_H
