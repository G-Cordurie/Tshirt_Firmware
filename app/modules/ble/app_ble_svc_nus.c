#include "ble_nus.h"
#include "nrf_ble_gatt.h"

#include "app_ble_gap.h"

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT); // BLE NUS service instance.

// Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module.
static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;

ble_nus_t *ble_nus_svc_instance_get(void)
{
    return (ble_nus_t *)&m_nus;
}

void nrf_ble_nus_on_gatt_evt(nrf_ble_gatt_evt_t const *p_gatt_evt)
{
    if (p_gatt_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        m_ble_nus_max_data_len = p_gatt_evt->params.att_mtu_effective - OVERHEAD_LENGTH;
    }
}

uint16_t ble_nus_max_date_len_get(void)
{
    return m_ble_nus_max_data_len & (uint16_t)0xFF;
}