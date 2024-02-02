#ifndef APP_BLE_SVC_BMS_H
#define APP_BLE_SVC_BMS_H

#include <stdint.h>

#include "nrf_ble_bms.h"

void           app_ble_svc_bms_init(void);
nrf_ble_bms_t *app_ble_svc_bms_instance_get(void);
void           delete_disconnected_bonds(void);
bool           delete_bonds_pending(void);
bool           delete_all_bonds_pending(void);
void           delete_all_bonds_complete(void);
ret_code_t     delete_all_bonds(void);
void           delete_all_except_requesting_bond(nrf_ble_bms_t const *p_bms);

#endif // APP_BLE_SVC_BMS_H