#include "ble_conn_state.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "sdk_errors.h"

#include "app_ble_gap.h"
#include "app_ble_mngr.h"
#include "app_ble_svc_bms.h"
#include "app_config.h"
#include "debug.h"

#define LESC_DEBUG_MODE           0 /**< Set to 1 to use LESC debug keys, allows you to use a sniffer to inspect traffic. */

#define SEC_PARAM_BOND            1                    /**< Perform bonding. */
#define SEC_PARAM_MITM            0                    /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC            1                    /**< LE Secure Connections enabled. */
#define SEC_PARAM_KEYPRESS        0                    /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES BLE_GAP_IO_CAPS_NONE /**< Display Only. */
#define SEC_PARAM_OOB             0                    /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE    7                    /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE    16                   /**< Maximum encryption key size. */

#define SEC_PASSKEY               "000000"

// static pm_peer_id_t m_peer_to_be_deleted = PM_PEER_ID_INVALID;

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const *p_evt)
{
    ret_code_t err_code;

    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
    case PM_EVT_CONN_SEC_START:
    {
        INFO("A security procedure has started...");
    }
    break;

#if !APP_MODULE_ENABLED(BLE_SEC)
    case PM_EVT_CONN_SEC_PARAMS_REQ:
    {
        err_code = pm_conn_sec_params_reply(p_evt->conn_handle, NULL, p_evt->params.conn_sec_params_req.p_context);
        APP_ERROR_CHECK(err_code);
    }
    break;
#endif

    case PM_EVT_CONN_SEC_SUCCEEDED:
    {
        pm_conn_sec_status_t conn_sec_status;

        // Check if the link is authenticated (meaning at least MITM).
        err_code = pm_conn_sec_status_get(p_evt->conn_handle, &conn_sec_status);
        APP_ERROR_CHECK(err_code);

        if (conn_sec_status.mitm_protected)
        {
            INFO("Link secured. Role: %d. conn_handle: %d, Procedure: %d", ble_conn_state_role(p_evt->conn_handle), p_evt->conn_handle,
                 p_evt->params.conn_sec_succeeded.procedure);
        }
        else
        {
            // The peer did not use MITM, disconnect.
            INFO("The peer did not use MITM");
            // INFO("The peer did not use MITM, disconnecting");
            // uint16_t conn_handle = ble_conn_handle_get();
            // err_code             = pm_peer_id_get(conn_handle, &m_peer_to_be_deleted);
            // APP_ERROR_CHECK(err_code);
            // err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            // APP_ERROR_CHECK(err_code);
        }

        if (pm_peer_count() > 1)
        {
            delete_all_except_requesting_bond(app_ble_svc_bms_instance_get());
        }
    }
    break;

    case PM_EVT_CONN_SEC_FAILED:
        ble_conn_handle_update(BLE_CONN_HANDLE_INVALID);
        break;

    case PM_EVT_PEER_DELETE_SUCCEEDED:
        if (!delete_bonds_pending() && !delete_all_bonds_pending())
        {
            // No more peers are flagged for deletion and we are not going to delete all peers.
            if (!is_ble_connected())
            {
                (void)advertising_start(false);
            }
        }
        break;

    case PM_EVT_PEERS_DELETE_SUCCEEDED:
        delete_all_bonds_complete();

        if (!is_ble_connected())
        {
            (void)advertising_start(false);
        }
        break;

    case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        if (p_evt->params.peer_data_update_succeeded.flash_changed &&
            (p_evt->params.peer_data_update_succeeded.data_id == PM_PEER_DATA_ID_BONDING))
        {
            INFO("New Bond");
        }
        break;

    case PM_EVT_CONN_SEC_CONFIG_REQ:
    {
        INFO("Accept pairing request from an already bonded peer.");
        pm_conn_sec_config_t conn_sec_config = {.allow_repairing = true};
        pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
    }
    break;

    case PM_EVT_BONDED_PEER_CONNECTED:
        INFO("PM_EVT_BONDED_PEER_CONNECTED");
        break;

    case PM_EVT_SERVICE_CHANGED_IND_SENT:
        INFO("Service changed indication has been sent");
        break;

    case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        INFO("Service changed indication confirmation");
        break;

    default:
        break;
    }
}

static void static_passkey_pair_init(void)
{
    uint32_t err_code;

    static uint8_t   passkey[] = SEC_PASSKEY;
    static ble_opt_t m_ble_opt;
    m_ble_opt.gap_opt.passkey.p_passkey = &passkey[0];

    err_code = sd_ble_opt_set(BLE_GAP_OPT_PASSKEY, &m_ble_opt);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Peer Manager initialization.
 */
void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);

    static_passkey_pair_init();

    // pm_local_database_has_changed();
}