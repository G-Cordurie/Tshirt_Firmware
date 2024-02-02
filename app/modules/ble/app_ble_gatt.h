#ifndef APP_BLE_GATT_H
#define APP_BLE_GATT_H

void gatt_init(void);
void gatt_mtu_set(uint16_t att_mtu);
void data_len_ext_set(uint16_t conn_handle, uint8_t data_length);

#endif // APP_BLE_GATT_H