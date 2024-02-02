#ifndef APP_BLE_SVC_BAS
#define APP_BLE_SVC_BAS

#include <stdint.h>

#include "sdk_errors.h"

void       battery_ble_init(void);
ret_code_t battery_power_set(uint8_t charge);
ret_code_t battery_level_set(uint8_t battery_level);
uint8_t    battery_level_last_get(void);

#endif // APP_BLE_SVC_BAS