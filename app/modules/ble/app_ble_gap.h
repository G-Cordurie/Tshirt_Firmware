#ifndef APP_BLE_GAP_H
#define APP_BLE_GAP_H

#include <stdbool.h>
#include <stdint.h>

#include "ble_advertising.h"
#include "ble_dfu.h"
#include "ble_gap.h"

#define L2CAP_HDR_LEN        4 // Length of a L2CAP header, in bytes.
#define DATA_LENGTH_MAX      (NRF_SDH_BLE_GATT_MAX_MTU_SIZE + L2CAP_HDR_LEN)

// #define OPCODE_LENGTH        1U
// #define HANDLE_LENGTH        2U
#define OVERHEAD_LENGTH      ((uint8_t)(1U + 2U)) // OPCODE_LENGTH + HANDLE_LENGTH

#define BLE_DEVICE_NAME      "DiagW-0000" // Name of device. Will be included in the advertising data
#define MANUFACTURER_NAME    "ChronoLife" // Manufacturer. Will be passed to Device Information Service.
#define COMPANY_IDENTIFIER   0x436CUL
#define SW_REV_STR           "1.5.0-alpha.1-noSec-sp.15_b_gc1"

typedef struct
{
    uint16_t       att_mtu;                  // GATT ATT MTU, in bytes.
    uint16_t       conn_interval;            // Connection interval expressed in units of 1.25 ms.
    ble_gap_phys_t phys;                     // Preferred PHYs.
    bool           conn_evt_len_ext_enabled; // Connection event length extension status.
} ble_conn_config_t;

typedef enum ble_adv_status_tag
{
    ble_adv_status_unknown = 0,
    ble_adv_status_idle,
    ble_adv_status_ongoing,
} ble_adv_status_t;

extern ble_conn_config_t m_conn_config;

void             gap_params_init(void);
void             advertising_config_get(ble_adv_modes_config_t *p_config);
void             advertising_init(void);
ret_code_t       advertising_start(bool erase_bonds);
char const      *phy_str(ble_gap_phys_t phys);
void             conn_params_init(void);
void             device_name_base36_set(void);
void             device_name_base10_set(uint32_t value);
void             ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event);
void             tx_power_set(void);
void             advertising_stop(void);
void             ble_disconnect(void);
void             conn_evt_len_ext_set(bool status);
void             preferred_phy_set(ble_gap_phys_t *p_phy);
void             advertising_modes_config_set(ble_adv_modes_config_t *p_config);
ble_adv_status_t advertising_status_get(void);
void             advertising_status_set(ble_adv_status_t status);

#endif // APP_BLE_GAP_H