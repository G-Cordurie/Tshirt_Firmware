#include "app_error.h"
#include "ble_conn_state.h"
#include "nrf_ble_bms.h"
#include "peer_manager.h"

#include "app_ble_svc_qwr.h"
#include "debug.h"

NRF_BLE_BMS_DEF(m_bms); // Bond Management service instance.

static ble_conn_state_user_flag_id_t m_bms_bonds_to_delete; // Flags used to identify bonds that should be deleted.

// Flag to show that advertising should not be started after a single peer is deleted (@ref PM_EVT_PEER_DELETE_SUCCEEDED) because
// we are waiting for ALL peers to be deleted (@ref PM_EVT_PEERS_DELETE_SUCCEEDED).
static bool m_delete_all_bonds_pending = false;

#if USE_AUTHORIZATION_CODE
static uint8_t m_auth_code[]   = {'C', 'L', 'I', 'F'}; // 0x43, 0x4C, 0x49, 0x46
static int     m_auth_code_len = sizeof(m_auth_code);
#endif

/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling events from bond management service.
 */
static void bms_evt_handler(nrf_ble_bms_t *p_ess, nrf_ble_bms_evt_t *p_evt)
{
    ret_code_t err_code;
    bool       is_authorized = true;

    switch (p_evt->evt_type)
    {
    case NRF_BLE_BMS_EVT_AUTH:
        INFO("Authorization request.");
#if USE_AUTHORIZATION_CODE
        if ((p_evt->auth_code.len != m_auth_code_len) || (memcmp(m_auth_code, p_evt->auth_code.code, m_auth_code_len) != 0))
        {
            is_authorized = false;
        }
#endif
        err_code = nrf_ble_bms_auth_response(&m_bms, is_authorized);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for marking the requester's bond for deletion.
 */
static void delete_requesting_bond_callback(nrf_ble_bms_t const *p_bms)
{
    ret_code_t   err_code;
    pm_peer_id_t peer_id;

    INFO("Client requested that bond to current device deleted");

    err_code = pm_peer_id_get(p_bms->conn_handle, &peer_id);
    APP_ERROR_CHECK(err_code);

    if (peer_id != PM_PEER_ID_INVALID)
    {
        ble_conn_state_user_flag_set(p_bms->conn_handle, m_bms_bonds_to_delete, true);
    }
    else
    {
        INFO("Current device is not bonded!");
    }
}

/**@brief Function for deleting a single bond if it does not belong to a connected peer.
 *
 * This will mark the bond for deferred deletion if the peer is connected.
 */
static void bond_delete(uint16_t conn_handle, void *p_context)
{
    UNUSED_PARAMETER(p_context);
    ret_code_t   err_code;
    pm_peer_id_t peer_id;

    if (ble_conn_state_status(conn_handle) == BLE_CONN_STATUS_CONNECTED)
    {
        ble_conn_state_user_flag_set(conn_handle, m_bms_bonds_to_delete, true);
    }
    else
    {
        INFO("Attempting to delete bond.");
        err_code = pm_peer_id_get(conn_handle, &peer_id);
        APP_ERROR_CHECK(err_code);
        if (peer_id != PM_PEER_ID_INVALID)
        {
            err_code = pm_peer_delete(peer_id);
            APP_ERROR_CHECK(err_code);
            ble_conn_state_user_flag_set(conn_handle, m_bms_bonds_to_delete, false);
        }
    }
}

/**@brief Function for deleting all bonds
 */
static void delete_all_bonds_callback(nrf_ble_bms_t const *p_bms)
{
    ret_code_t err_code;
    uint16_t   conn_handle;

    INFO("Client requested that all bonds be deleted");

    pm_peer_id_t peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    while (peer_id != PM_PEER_ID_INVALID)
    {
        err_code = pm_conn_handle_get(peer_id, &conn_handle);
        APP_ERROR_CHECK(err_code);

        if (conn_handle != BLE_CONN_HANDLE_INVALID)
        {
            /* Defer the deletion since this connection is active. */
            INFO("Defer the deletion");
            ble_conn_state_user_flag_set(conn_handle, m_bms_bonds_to_delete, true);
        }
        else
        {
            err_code = pm_peer_delete(peer_id);
            APP_ERROR_CHECK(err_code);
            ble_conn_state_user_flag_set(conn_handle, m_bms_bonds_to_delete, false);
        }

        peer_id = pm_next_peer_id_get(peer_id);
    }
}

/**@brief Function for deleting all bet requesting device bonds
 */
void delete_all_except_requesting_bond(nrf_ble_bms_t const *p_bms)
{
    ret_code_t err_code;
    uint16_t   conn_handle;

    INFO("Client requested that all bonds except current bond be deleted");

    pm_peer_id_t peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    while (peer_id != PM_PEER_ID_INVALID)
    {
        err_code = pm_conn_handle_get(peer_id, &conn_handle);
        APP_ERROR_CHECK(err_code);

        /* Do nothing if this is our own bond. */
        if (conn_handle != p_bms->conn_handle)
        {
            err_code = pm_peer_delete(peer_id);
            APP_ERROR_CHECK(err_code);
            ble_conn_state_user_flag_set(conn_handle, m_bms_bonds_to_delete, false);
        }

        peer_id = pm_next_peer_id_get(peer_id);
    }
}

/**@brief Function for performing deferred deletions.
 */
void delete_disconnected_bonds(void)
{
    uint32_t n_calls = ble_conn_state_for_each_set_user_flag(m_bms_bonds_to_delete, bond_delete, NULL);
    UNUSED_RETURN_VALUE(n_calls);
}

/** @brief Clear bonding information from persistent storage.
 */
ret_code_t delete_all_bonds(void)
{
    INFO("Erase bonds!");
    m_delete_all_bonds_pending = true;
    return pm_peers_delete();
}

bool delete_all_bonds_pending(void)
{
    return m_delete_all_bonds_pending;
}

void delete_all_bonds_complete(void)
{
    m_delete_all_bonds_pending = false;
}

/**@brief Function for determening if there are one or more connections with bonds that are flagged
 *        for deletion.
 */
bool delete_bonds_pending(void)
{
    ble_conn_state_conn_handle_list_t conn_handle_list = ble_conn_state_conn_handles();

    for (uint32_t i = 0; i < conn_handle_list.len; i++)
    {
        uint16_t conn_handle = conn_handle_list.conn_handles[i];
        bool     pending     = ble_conn_state_user_flag_get(conn_handle, m_bms_bonds_to_delete);

        if (pending == true)
        {
            return pending;
        }
    }
    return false;
}

void app_ble_svc_bms_init(void)
{
    ret_code_t         err_code;
    nrf_ble_bms_init_t bms_init;

    // Initialize Bond Management Service
    memset(&bms_init, 0, sizeof(bms_init));

    m_bms_bonds_to_delete  = ble_conn_state_user_flag_acquire();
    bms_init.evt_handler   = bms_evt_handler;
    bms_init.error_handler = service_error_handler;
#if USE_AUTHORIZATION_CODE
    bms_init.feature.delete_requesting_auth         = true;
    bms_init.feature.delete_all_auth                = true;
    bms_init.feature.delete_all_but_requesting_auth = true;
#else
    bms_init.feature.delete_requesting         = true;
    bms_init.feature.delete_all                = true;
    bms_init.feature.delete_all_but_requesting = true;
#endif
    bms_init.bms_feature_sec_req = SEC_OPEN;
    bms_init.bms_ctrlpt_sec_req  = SEC_OPEN;

    nrf_ble_qwr_t *p_qwr                                 = app_ble_svc_qwr_instance_get();
    bms_init.p_qwr                                       = p_qwr;
    bms_init.bond_callbacks.delete_requesting            = delete_requesting_bond_callback;
    bms_init.bond_callbacks.delete_all                   = delete_all_bonds_callback;
    bms_init.bond_callbacks.delete_all_except_requesting = delete_all_except_requesting_bond;

    err_code = nrf_ble_bms_init(&m_bms, &bms_init);
    APP_ERROR_CHECK(err_code);
}

nrf_ble_bms_t *app_ble_svc_bms_instance_get(void)
{
    return &m_bms;
}