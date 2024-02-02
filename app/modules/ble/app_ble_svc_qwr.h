#ifndef APP_BLE_SVC_QWR_H
#define APP_BLE_SVC_QWR_H

#include <stdint.h>

#include "nrf_ble_qwr.h"

void           app_ble_svc_qwr_init(void);
void           app_ble_svc_qwr_conn_handle_assign(uint16_t conn_handle);
nrf_ble_qwr_t *app_ble_svc_qwr_instance_get(void);

#endif // APP_BLE_SVC_QWR_H