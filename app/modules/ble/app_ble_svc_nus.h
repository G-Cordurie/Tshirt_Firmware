#ifndef APP_BLE_SVC_NUS_H
#define APP_BLE_SVC_NUS_H

#include "ble_nus.h"
#include "nrf_ble_gatt.h"

#define NUS_SERVICE_UUID_TYPE BLE_UUID_TYPE_VENDOR_BEGIN // UUID type for the Nordic UART Service (vendor specific).

ble_nus_t *ble_nus_svc_instance_get(void);
void       nrf_ble_nus_on_gatt_evt(nrf_ble_gatt_evt_t const *p_gatt_evt);
uint16_t   ble_nus_max_date_len_get(void);

#endif