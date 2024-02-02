#ifndef APP_BLE_MNGR_H
#define APP_BLE_MNGR_H

#include <stdint.h>

#define APP_BLE_CMD_ID_LEN            1U
#define DATA_SESSION_SET_CMD_LEN      14U
#define TEMP_CALIB_STATUS_SET_CMD_LEN 4U
#define APP_BLE_CMD_MAX_LEN           (uint16_t)(APP_BLE_CMD_ID_LEN + DATA_SESSION_SET_CMD_LEN)

typedef enum ble_cmd_id_tag
{
    cmd_id_no_command            = 0x00U,
    cmd_id_device_shutdwon       = 0x05U,
    cmd_id_imp_meas_start        = 0x06U,
    cmd_id_device_name_set       = 0x07U,
    cmd_id_temp_calib_status_set = 0x0BU,
    cmd_id_auto_diag_status_set  = 0x10U,
    cmd_id_data_session_reset    = 0x11U,
    cmd_id_data_session_set      = 0x12U,
    cmd_id_user_id_reset         = 0x13U,
} ble_cmd_id_t;

void     ble_stack_init(void);
void     ble_stack_config(void);
void     ble_stack_disable(void);
bool     is_ble_connected(void);
void     ble_conn_handle_update(uint16_t conn_handle);
uint16_t ble_conn_handle_get(void);
void     app_ble_task(void);
void     app_ble_cmd_update_callback(ble_cmd_id_t cmd_id, const uint8_t *cmd_data, uint16_t cmd_len);
bool     app_ble_cmd_is_pending(void);
void     app_ble_cmd_cancel(ble_cmd_id_t cmd_id);

#endif // APP_BLE_MNGR_H