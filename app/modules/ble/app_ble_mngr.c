#include <string.h>

#include "app_error.h"
#include "ble.h"
#include "ble_dfu.h"
#include "ble_dis.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "ble_nus.h"
#include "ble_srv_common.h"
#include "ble_types.h"
#include "nordic_common.h"
#include "nrf_ble_qwr.h"
#include "nrf_log.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_soc.h"
#include "peer_manager_handler.h"
#include "sdk_config.h"

#include "app_ble_gap.h"
#include "app_ble_gatt.h"
#include "app_ble_mngr.h"
#include "app_ble_sm.h"
#include "app_ble_svc_bas.h"
#include "app_ble_svc_bms.h"
#include "app_ble_svc_nus.h"
#include "app_ble_svc_qwr.h"
#include "app_config.h"
#include "battery.h"
#include "battery_chrgr.h"
#include "ble_diagw.h"
#include "breath.h"
#include "data_acq.h"
#include "data_session.h"
#include "debug.h"
#include "pwr_mngr.h"
#include "spi_flash.h"
#include "storage.h"
#include "temp.h"
#include "utils.h"

#define APP_BLE_CONN_CFG_TAG      1U // A tag identifying the SoftDevice BLE configuration.
#define APP_BLE_OBSERVER_PRIO     3  // Application's BLE observer priority. You shouldn't need to modify this value.
#define APP_FEATURE_NOT_SUPPORTED BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2 // Reply when unsupported features are requested.

typedef enum
{
    SD_STATE_IDLE,     // No state change requested by the app to the SoftDevice.
    SD_STATE_DISABLED, // Softdevice was disabled by app request
    SD_STATE_ENABLED,  // Softdevice was enabled by app request
} sd_state_t;

typedef struct ble_cmd_tag
{
    ble_cmd_id_t id;
    uint8_t      len;
    uint8_t      data[APP_BLE_CMD_MAX_LEN];
} ble_cmd_t;

volatile uint16_t         m_conn_handle = BLE_CONN_HANDLE_INVALID; // Handle of the current connection.
static volatile ble_cmd_t m_ble_cmd     = {.id = 0, .len = 0, .data = {0}};

static void sd_state_evt_handler(nrf_sdh_state_evt_t state, void *p_context);

NRF_SDH_STATE_OBSERVER(m_sd_state_observer, 0) = {
    .handler   = sd_state_evt_handler,
    .p_context = NULL,
};

static volatile sd_state_t m_sd_state = SD_STATE_IDLE;

static void sd_pwr_mode_init(void)
{
    ret_code_t err_code;

    err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
    APP_ERROR_CHECK(err_code);

    err_code = sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief SoftDevice enable/disable state handler.
 *
 * @param[in] state     State.
 * @param[in] p_context Context.
 */
static void sd_state_evt_handler(nrf_sdh_state_evt_t state, void *p_context)
{
    switch (state)
    {
    case NRF_SDH_EVT_STATE_ENABLE_PREPARE:
        INFO("Enabling SD...");
        break;

    case NRF_SDH_EVT_STATE_ENABLED:
        m_sd_state = SD_STATE_ENABLED;
        INFO("SD is enabled");
        break;

    case NRF_SDH_EVT_STATE_DISABLED:
        m_sd_state = SD_STATE_DISABLED;
        INFO("SD disabled");
        break;

    default:
        break;
    }
}

void ble_stack_disable(void)
{
    uint32_t err_code;

    if (m_sd_state != SD_STATE_ENABLED)
    {
        INFO("Function: %s, SD is already disabled!", (uint32_t) __func__);
        return;
    }

    advertising_stop(); // Stop ongoing advertisement if any
    ble_disconnect();   // Disconnect if connected to a BLE central

    err_code = nrf_sdh_disable_request(); // Send SD disable request
    APP_ERROR_CHECK(err_code);

    while (m_sd_state != SD_STATE_DISABLED)
    {
        // Wait until softdevice is disabled
    }
}

/**
 * @brief Function for initializing services that will be used by the application.
 *
 */
void services_init(void)
{
    uint32_t err_code;

    app_ble_svc_qwr_init();

#if APP_MODULE_ENABLED(BLE_SEC)
    app_ble_svc_bms_init();
#endif

    battery_ble_init();

    // Initialize Device Information Service.
    ble_dis_init_t dis_init;
    memset(&dis_init, 0, sizeof(dis_init));
    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, (char *)HW_REV_STR);
    ble_srv_ascii_to_utf8(&dis_init.sw_rev_str, (char *)SW_REV_STR);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str, (char *)MODEL_NUMBER_STR);

    // BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    // BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);
    dis_init.dis_char_rd_sec = SEC_OPEN;
    err_code                 = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);

    // Initialize DiagW Service
    ble_diagw_init_t diagw_init;
    memset(&diagw_init, 0, sizeof(diagw_init));
    diagw_init.data_handler = diagw_data_handler;
    err_code                = ble_diagw_init(ble_diagw_svc_instance_get(), &diagw_init);
    APP_ERROR_CHECK(err_code);

    ble_dfu_buttonless_init_t dfus_init = {0};
    dfus_init.evt_handler               = ble_dfu_evt_handler;
    err_code                            = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);

#if APP_MODULE_ENABLED(APP_BLE_NUS)
    // Initialize NUS.
    ble_nus_init_t nus_init;
    memset(&nus_init, 0, sizeof(nus_init));
    err_code = ble_nus_init(ble_nus_svc_instance_get(), &nus_init);
    APP_ERROR_CHECK(err_code);
#endif
}

void ble_stack_config(void)
{
    if (m_sd_state != SD_STATE_ENABLED)
    {
        INFO("Function: %s, SD is not enabled!", (uint32_t) __func__);
        return;
    }

    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    gatt_mtu_set(m_conn_config.att_mtu);
    data_len_ext_set(BLE_CONN_HANDLE_INVALID, (NRF_SDH_BLE_GATT_MAX_MTU_SIZE + L2CAP_HDR_LEN));
    conn_evt_len_ext_set(m_conn_config.conn_evt_len_ext_enabled);
    preferred_phy_set(&m_conn_config.phys);
    tx_power_set();

    INFO("BLE stack config.");
}

static void on_ble_gap_evt_connected(ble_evt_t const *p_ble_evt, void *p_context)
{
    static bool first_ble_conn = true;

    m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    app_ble_svc_qwr_conn_handle_assign(m_conn_handle);
    nrf_ble_bms_set_conn_handle(app_ble_svc_bms_instance_get(), m_conn_handle);
    advertising_status_set(ble_adv_status_idle);

    if (first_ble_conn)
    {
        first_ble_conn = false;
        pm_disallow_device_to_sleep();

        ble_adv_modes_config_t config;
        advertising_config_get(&config);
        config.ble_adv_on_disconnect_disabled = true;
        config.ble_adv_slow_timeout           = BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED;
        advertising_modes_config_set(&config);
    }

    storage_percent_tmr_start(MEM_PERCENT_MODE_FAST);
    data_session_timeout_tmr_start();
#if APP_MODULE_ENABLED(APP_BLE_NUS)
    log_tmr_start();
#endif
    LED_ON();
}

static void on_ble_gap_evt_disconnected(ble_evt_t const *p_ble_evt, void *p_context)
{
    advertising_status_set(ble_adv_status_unknown);

    if (delete_bonds_pending())
    {
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED or PM_EVT_PEERS_DELETE_SUCCEEDED event.
        delete_disconnected_bonds();
    }
    else
    {
        (void)advertising_start(false);
    }

    m_conn_handle = BLE_CONN_HANDLE_INVALID;
    storage_percent_tmr_stop();

    app_ble_cmd_cancel(cmd_id_data_session_set);
    data_session_user_disconnect();

#if APP_MODULE_ENABLED(APP_BLE_NUS)
    log_tmr_stop();
#endif
    LED_OFF();
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        on_ble_gap_evt_connected(p_ble_evt, p_context);
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        on_ble_gap_evt_disconnected(p_ble_evt, p_context);
        break;

    case BLE_GAP_EVT_CONN_PARAM_UPDATE:
    {
        ble_gap_conn_params_t conn_param;

        memset(&conn_param, 0, sizeof(ble_gap_conn_params_t));
        conn_param.min_conn_interval = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.min_conn_interval;
        conn_param.max_conn_interval = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.max_conn_interval;
        UNUSED_VARIABLE(conn_param);

        mem_dump_on_conn_param_update_callback(UNITS_TO_MSEC(conn_param.min_conn_interval, UNIT_1_25_MS));

        // INFO("[%s] Connection interval updated: 0x%x, 0x%x", (uint32_t) __func__, conn_param.min_conn_interval,
        //      conn_param.max_conn_interval);

        INFO("Connection interval updated: %u ms, %u ms", UNITS_TO_MSEC(conn_param.min_conn_interval, UNIT_1_25_MS),
             UNITS_TO_MSEC(conn_param.max_conn_interval, UNIT_1_25_MS));
    }
    break;

    case BLE_GAP_EVT_PHY_UPDATE:
    {
        ble_gap_evt_phy_update_t const *p_phy_evt = &p_ble_evt->evt.gap_evt.params.phy_update;

        if (p_phy_evt->status == BLE_HCI_STATUS_CODE_LMP_ERROR_TRANSACTION_COLLISION)
        {
            // Ignore LL collisions.
            INFO("[%s] LL transaction collision during PHY update.", (uint32_t) __func__);
            break;
        }

        ble_gap_phys_t phys = {0};
        phys.tx_phys        = p_phy_evt->tx_phy;
        phys.rx_phys        = p_phy_evt->rx_phy;
        UNUSED_VARIABLE(phys);
        INFO("[%s] PHY update %s. PHY set to %s.", (uint32_t) __func__,
             (p_phy_evt->status == BLE_HCI_STATUS_CODE_SUCCESS) ? "accepted" : "rejected", phy_str(phys));
    }
    break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
    {
        INFO("[%s] Send PHY update response: %s", (uint32_t) __func__, phy_str(m_conn_config.phys));
        err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &m_conn_config.phys);
        APP_ERROR_CHECK(err_code);
    }
    break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        // Pairing not supported
        // err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
        // APP_ERROR_CHECK(err_code);
        INFO("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
        break;

    case BLE_GAP_EVT_PASSKEY_DISPLAY:
    {
        char passkey[PASSKEY_LENGTH + 1];
        memcpy(passkey, p_ble_evt->evt.gap_evt.params.passkey_display.passkey, PASSKEY_LENGTH);
        passkey[PASSKEY_LENGTH] = 0;

        INFO("[BLE_GAP_EVT_PASSKEY_DISPLAY] Passkey: %s", passkey);
    }
    break;

    case BLE_GAP_EVT_AUTH_KEY_REQUEST:
        INFO("BLE_GAP_EVT_AUTH_KEY_REQUEST");
        break;

    case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
        // INFO("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
        break;

    case BLE_GAP_EVT_AUTH_STATUS:
    {
        INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
             p_ble_evt->evt.gap_evt.params.auth_status.auth_status, p_ble_evt->evt.gap_evt.params.auth_status.bonded,
             p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4, *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
             *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));

        uint8_t auth_status = p_ble_evt->evt.gap_evt.params.auth_status.auth_status;
        bool    bonded      = p_ble_evt->evt.gap_evt.params.auth_status.bonded;

        if ((auth_status == BLE_GAP_SEC_STATUS_SUCCESS) && bonded)
        {
            // pm_local_database_has_changed();
            ble_disconnect();
        }
    }
    break;

    case BLE_GAP_EVT_CONN_SEC_UPDATE:
        INFO("BLE_GAP_EVT_CONN_SEC_UPDATE: Security mode: %u. Security level: %u",
             p_ble_evt->evt.gap_evt.params.conn_sec_update.conn_sec.sec_mode.sm,
             p_ble_evt->evt.gap_evt.params.conn_sec_update.conn_sec.sec_mode.lv);
        break;

    case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
    {
        uint16_t const mtu_requested = p_ble_evt->evt.gatts_evt.params.exchange_mtu_request.client_rx_mtu;
        uint16_t       mtu_reply;

        if (mtu_requested < NRF_SDH_BLE_GATT_MAX_MTU_SIZE)
        {
            mtu_reply = mtu_requested;
        }
        else
        {
            mtu_reply = NRF_SDH_BLE_GATT_MAX_MTU_SIZE;
        }

        NRF_LOG_DEBUG("Received BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST (request: %d, reply: %d).", mtu_requested, mtu_reply);

        err_code = sd_ble_gatts_exchange_mtu_reply(m_conn_handle, mtu_reply);
        // APP_ERROR_CHECK(err_code);
    }
    break;

    case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
    {
        ble_gap_data_length_params_t const dl_params = {
            .max_rx_octets  = NRF_SDH_BLE_GATT_MAX_MTU_SIZE + L2CAP_HDR_LEN,
            .max_tx_octets  = NRF_SDH_BLE_GATT_MAX_MTU_SIZE + L2CAP_HDR_LEN,
            .max_tx_time_us = BLE_GAP_DATA_LENGTH_AUTO,
            .max_rx_time_us = BLE_GAP_DATA_LENGTH_AUTO,
        };

        INFO("[%s] Send DLE update response: max_rx: %u, max_tx: %u", (uint32_t) __func__, dl_params.max_rx_octets,
             dl_params.max_tx_octets);
        err_code = sd_ble_gap_data_length_update(p_ble_evt->evt.gatts_evt.conn_handle, &dl_params, NULL);
        APP_ERROR_CHECK(err_code);
    }
    break;

    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        // No system attributes have been stored.
        err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GATTC_EVT_TIMEOUT:
        // Disconnect on GATT Client timeout event.
        NRF_LOG_DEBUG("GATT Client Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GATTS_EVT_TIMEOUT:
        // Disconnect on GATT Server timeout event.
        INFO("GATT Server Timeout! Disconnect..");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_EVT_USER_MEM_REQUEST:
        err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
    {
        ble_gatts_evt_rw_authorize_request_t  req;
        ble_gatts_rw_authorize_reply_params_t auth_reply;

        req = p_ble_evt->evt.gatts_evt.params.authorize_request;

        if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
        {
            if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ) || (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
            {
                if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                {
                    auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                }
                else
                {
                    auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                }
                auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                err_code                            = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle, &auth_reply);
                APP_ERROR_CHECK(err_code);
            }
        }
        break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST
    }

    default:
        // No implementation needed.
        break;
    }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
void ble_stack_init(void)
{
    ret_code_t err_code;

    if (m_sd_state == SD_STATE_ENABLED)
    {
        INFO("Function: %s, SD is already enabled!", (uint32_t) __func__);
        return;
    }

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code           = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Overwrite some of the default configurations for the BLE stack.
    ble_cfg_t ble_cfg;
    memset(&ble_cfg, 0, sizeof(ble_cfg));

    // Three vendor-specific UUIDs: our own, Nordic's DFU service and Nordic's
    // UART service.
    ble_cfg.common_cfg.vs_uuid_cfg.vs_uuid_count = NRF_SDH_BLE_VS_UUID_COUNT;
    err_code                                     = sd_ble_cfg_set(BLE_COMMON_CFG_VS_UUID, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);

    while (m_sd_state != SD_STATE_ENABLED)
    {
        // Wait until softdevice is enabled...
    }

    sd_pwr_mode_init();
}

bool is_ble_connected(void)
{
    return m_conn_handle != BLE_CONN_HANDLE_INVALID;
}

void ble_conn_handle_update(uint16_t conn_handle)
{
    m_conn_handle = conn_handle;
}

uint16_t ble_conn_handle_get(void)
{
    return m_conn_handle;
}

void app_ble_cmd_update_callback(ble_cmd_id_t cmd_id, const uint8_t *cmd_data, uint16_t cmd_len)
{
    m_ble_cmd.id  = cmd_id;
    m_ble_cmd.len = cmd_len;
    if (cmd_data != NULL && cmd_len > 0)
    {
        memcpy((uint8_t *)m_ble_cmd.data, cmd_data, cmd_len);
    }
}

bool app_ble_cmd_is_pending(void)
{
    return m_ble_cmd.id != cmd_id_no_command;
}

void app_ble_cmd_cancel(ble_cmd_id_t cmd_id)
{
    CRITICAL_REGION_ENTER();
    {
        if (app_ble_cmd_is_pending() && m_ble_cmd.id == cmd_id)
        {
            m_ble_cmd.id = cmd_id_no_command;
        }
    }
    CRITICAL_REGION_EXIT();
}

void app_ble_task(void)
{
    if (app_ble_cmd_is_pending())
    {
        CRITICAL_REGION_ENTER();
        {
            switch (m_ble_cmd.id)
            {
            case cmd_id_data_session_set:
            {
                if (m_ble_cmd.len >= DATA_SESSION_SET_CMD_LEN)
                {
                    uint64_t timestamp;
                    uint64_t user_id;
                    memcpy((uint8_t *)&timestamp, (uint8_t *)m_ble_cmd.data, sizeof(timestamp));
                    memcpy((uint8_t *)&user_id, (uint8_t *)&m_ble_cmd.data[sizeof(timestamp)], USER_ID_LEN);
                    data_session_update(user_id & 0xFFFFFFFFFFFF, timestamp);
                }
            }
            break;

            case cmd_id_data_session_reset:
            {
                data_session_reset();
                pm_allow_device_to_sleep();
            }
            break;

            case cmd_id_user_id_reset:
            {
                data_session_user_id_reset();
                pm_allow_device_to_sleep();
            }
            break;

            case cmd_id_temp_calib_status_set:
            {
                if (m_ble_cmd.len >= TEMP_CALIB_STATUS_SET_CMD_LEN)
                {
                    uint32_t status = *(uint32_t *)&m_ble_cmd.data;
                    temp_calib_set_status(status);
                    temp_system_reset();
                }
            }
            break;

            default:
                break;
            }

            m_ble_cmd.id = cmd_id_no_command;
        }
        CRITICAL_REGION_EXIT();
    }

#if (HW_TYPE == HW_TYPE_0706)
    static bool battery_was_charging = false;

    if (battery_is_charging())
    {
        battery_was_charging = true;
        ble_disconnect();
        if (advertising_status_get() == ble_adv_status_ongoing)
        {
            advertising_stop();
        }
    }
    else if (battery_was_charging)
    {
        if (is_ble_connected() || advertising_start(false) == NRF_SUCCESS)
        {
            battery_was_charging = false;
        }
    }
#endif
}