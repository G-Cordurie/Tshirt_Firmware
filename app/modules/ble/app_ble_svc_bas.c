#include <stdint.h>
#include <string.h>

#include "app_error.h"
#include "ble_bas.h"
#include "ble_types.h"
#include "nrf_error.h"
#include "sdk_errors.h"

BLE_BAS_DEF(m_bas); // BLE Battery Service instance

#define BLE_UUID_BATTERY_POWER_STATE_CHAR 0x2A1A // Battery Level characteristic UUID.

#if 0
static uint8_t battery_char_value(uint8_t charging)
{
    // We need to generate a byte such that each bit couple
    // encodes some power charge information
    //
    // In order of definition, those couple of bytes as we
    // implement them mean:
    // Battery Present
    // Discharging
    // Charging/or not
    // Not supported (power state flag)

    if (charging)
    {
        charging = 3;
    }
    else
    {
        charging = 2;
    }

    return (3 | (3 << 2) | (charging << 4) | (1 << 6));
}

/**@brief Function for adding the Battery Level characteristic.
 *
 * @param[in]   p_bas        Battery Service structure.
 * @param[in]   p_bas_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t battery_power_char_add(ble_bas_t *p_bas, const ble_bas_init_t *p_bas_init)
{
    uint32_t err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t initial_battery_power;
    uint8_t encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];
    uint8_t init_len;

    // Add Battery Level characteristic
    if (p_bas->is_notification_supported)
    {
        memset(&cccd_md, 0, sizeof(cccd_md));

        // According to BAS_SPEC_V10, the read operation on cccd should be possible without
        // authentication.
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
        cccd_md.write_perm = p_bas_init->battery_level_char_attr_md.cccd_write_perm;
        cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    }

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read = 1;
    char_md.char_props.notify = (p_bas->is_notification_supported) ? 1 : 0;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = (p_bas->is_notification_supported) ? &cccd_md : NULL;
    char_md.p_sccd_md = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_BATTERY_POWER_STATE_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm = p_bas_init->battery_level_char_attr_md.read_perm;
    attr_md.write_perm = p_bas_init->battery_level_char_attr_md.write_perm;
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    initial_battery_power = battery_char_value(last_battery_power);

    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len = sizeof(uint8_t);
    attr_char_value.p_value = &initial_battery_power;

    err_code = sd_ble_gatts_characteristic_add(p_bas->service_handle, &char_md,
                                               &attr_char_value,
                                               &battery_power_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    if (p_bas_init->p_report_ref != NULL)
    {
        // Add Report Reference descriptor
        BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_REPORT_REF_DESCR);

        memset(&attr_md, 0, sizeof(attr_md));

        attr_md.read_perm = p_bas_init->battery_level_report_read_perm;
        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

        attr_md.vloc = BLE_GATTS_VLOC_STACK;
        attr_md.rd_auth = 0;
        attr_md.wr_auth = 0;
        attr_md.vlen = 0;

        init_len = ble_srv_report_ref_encode(encoded_report_ref, p_bas_init->p_report_ref);

        memset(&attr_char_value, 0, sizeof(attr_char_value));

        attr_char_value.p_uuid = &ble_uuid;
        attr_char_value.p_attr_md = &attr_md;
        attr_char_value.init_len = init_len;
        attr_char_value.init_offs = 0;
        attr_char_value.max_len = attr_char_value.init_len;
        attr_char_value.p_value = encoded_report_ref;

        err_code = sd_ble_gatts_descriptor_add(battery_power_handle.value_handle,
                                               &attr_char_value,
                                               &p_bas->report_ref_handle);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }
    else
    {
        p_bas->report_ref_handle = BLE_GATT_HANDLE_INVALID;
    }

    return NRF_SUCCESS;
}

uint32_t ble_bas_battery_power_update(ble_bas_t *p_bas, uint8_t battery_power)
{
    if (p_bas == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    if (battery_power != last_battery_power)
    {
        // Initialize value struct.
        memset(&gatts_value, 0, sizeof(gatts_value));

        gatts_value.len = sizeof(uint8_t);
        gatts_value.offset = 0;
        gatts_value.p_value = &battery_power;

        // Update database.
        err_code = sd_ble_gatts_value_set(p_bas->conn_handle,
                                          battery_power_handle.value_handle,
                                          &gatts_value);
        if (err_code == NRF_SUCCESS)
        {
            // Save new battery value.
            last_battery_power = battery_power;
        }
        else
        {
            return err_code;
        }

        // Send value if connected and notifying.
        if ((p_bas->conn_handle != BLE_CONN_HANDLE_INVALID) && p_bas->is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;

            memset(&hvx_params, 0, sizeof(hvx_params));

            hvx_params.handle = battery_power_handle.value_handle;
            hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = gatts_value.offset;
            hvx_params.p_len = &gatts_value.len;
            hvx_params.p_data = gatts_value.p_value;

            err_code = sd_ble_gatts_hvx(p_bas->conn_handle, &hvx_params);
        }
        else
        {
            err_code = NRF_ERROR_INVALID_STATE;
        }
    }

    return err_code;
}
#endif

void battery_ble_init(void)
{
    ble_bas_init_t bas_init;
    ret_code_t     err_code;

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    // BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    // BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    // BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);
    // BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    // Here the sec level for the Battery Service can be changed/increased.
    bas_init.bl_rd_sec        = SEC_OPEN;
    bas_init.bl_cccd_wr_sec   = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // err_code = battery_power_char_add(&m_bas, &bas_init);
    // APP_ERROR_CHECK(err_code);
}

ret_code_t battery_level_set(uint8_t battery_level)
{
    return ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
}

uint8_t battery_level_last_get(void)
{
    return m_bas.battery_level_last;
}

ret_code_t battery_power_set(uint8_t charge)
{
    // uint8_t battery_power = battery_char_value(charge);
    // return ble_bas_battery_power_update(&m_bas, battery_power);
    return NRF_SUCCESS;
}