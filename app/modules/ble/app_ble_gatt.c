#include <stdbool.h>

#include "app_error.h"
#include "nrf_ble_gatt.h"
#include "nrf_sdh_ble.h"

#include "app_ble_gap.h"
#include "app_ble_gatt.h"
#include "app_ble_svc_nus.h"
#include "ble_diagw.h"
#include "data_acq.h"
#include "debug.h"
#include "timer.h"
#include "utils.h"

NRF_BLE_GATT_DEF(m_gatt); // GATT module instance.

void data_len_ext_set(uint16_t conn_handle, uint8_t data_length)
{
    ret_code_t err_code = nrf_ble_gatt_data_length_set(&m_gatt, conn_handle, data_length);
    if (err_code != NRF_SUCCESS)
    {
        INFO("[%s] DLE update error %u !", (uint32_t) __func__, err_code);
    }
}

/**
 * @brief Function for handling events from the GATT library
 *
 * @param p_gatt
 * @param p_evt
 */
static void gatt_evt_handler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt)
{
    switch (p_evt->evt_id)
    {
    case NRF_BLE_GATT_EVT_ATT_MTU_UPDATED:
    {
        m_conn_config.att_mtu = p_evt->params.att_mtu_effective;

        INFO("[%s] ATT MTU exchange completed: %u (Central desired MTU %u, Peripheral desired MTU %u)", (uint32_t) __func__,
             nrf_ble_gatt_eff_mtu_get(&m_gatt, p_evt->conn_handle), p_gatt->att_mtu_desired_central, p_gatt->att_mtu_desired_periph);

        if (BLE_GATT_ATT_MTU_DEFAULT != p_evt->params.att_mtu_effective)
        {
            uint8_t dl = 0;
            (void)nrf_ble_gatt_data_length_get(&m_gatt, p_evt->conn_handle, &dl);

            INFO("[%s] Current data length: %u", (uint32_t) __func__, dl);

            if (dl != DATA_LENGTH_MAX)
            {
                data_len_ext_set(p_evt->conn_handle, DATA_LENGTH_MAX);
            }
        }
    }
    break;

    case NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED:
    {
        INFO("[%s] Data length updated to %u bytes", (uint32_t) __func__, p_evt->params.data_length);
    }
    break;

    default:
        break;
    }

    nrf_ble_diagw_on_gatt_evt(ble_diagw_svc_instance_get(), p_evt);
#if APP_MODULE_ENABLED(APP_BLE_NUS)
    nrf_ble_nus_on_gatt_evt(p_evt);
#endif
}

/**
 * @brief Function for initializing the GATT module.
 * @details The GATT module handles ATT_MTU and Data Length update procedures automatically.
 */
void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
}

void gatt_mtu_set(uint16_t att_mtu)
{
    m_conn_config.att_mtu = att_mtu;

    ret_code_t err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, att_mtu);
    APP_ERROR_CHECK(err_code);
}