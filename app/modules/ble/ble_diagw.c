#include <string.h>

#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_ble_gatt.h"
#include "peer_manager.h"
#include "sdk_common.h"

#include "app_ble_gap.h"
#include "app_ble_gatt.h"
#include "app_ble_mngr.h"
#include "app_config.h"
#include "app_error.h"
#include "auto_diag.h"
#include "ble_diagw.h"
#include "breath.h"
#include "data_acq.h"
#include "data_session.h"
#include "debug.h"
#include "ecg.h"
#include "imp.h"
#include "md.pb.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "pwr_mngr.h"
#include "storage.h"
#include "utils.h"

#define APP_FEATURE_NOT_SUPPORTED BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2 // Reply when unsupported features are requested.

BLE_DIAGW_DEF(m_diagws); // BLE diagw service instance.

#define BLE_TIMEOUT 100U

static volatile bool hvn_tx_busy = false;

static void diagw_notif_status_set(notif_data_type_t type, uint8_t status)
{
    switch (type)
    {
    case ECG_TYPE:
        m_diagws.diagw_notif.status.items.ecg = status;
        break;

    case BREATH_TYPE:
        m_diagws.diagw_notif.status.items.breath = status;
        break;

    case TEMP_TYPE:
        m_diagws.diagw_notif.status.items.temp = status;
        break;

    case IMP_TYPE:
        m_diagws.diagw_notif.status.items.imp = status;
        break;

    case ACC_TYPE:
        m_diagws.diagw_notif.status.items.acc = status;
        break;

    case MEM_DUMP_TYPE:
        m_diagws.diagw_notif.status.items.mem_dump = status;
        break;

    case MEM_DUMP_CTRLPT_TYPE:
        m_diagws.diagw_notif.status.items.mem_dump_ctrlpt = status;
        break;

    default:
        break;
    }
}

static uint8_t diagw_notif_status_get(notif_data_type_t type)
{
    switch (type)
    {
    case ECG_TYPE:
        return m_diagws.diagw_notif.status.items.ecg;

    case BREATH_TYPE:
        return m_diagws.diagw_notif.status.items.breath;

    case TEMP_TYPE:
        return m_diagws.diagw_notif.status.items.temp;

    case IMP_TYPE:
        return m_diagws.diagw_notif.status.items.imp;

    case ACC_TYPE:
        return m_diagws.diagw_notif.status.items.acc;

    case MEM_DUMP_TYPE:
        return m_diagws.diagw_notif.status.items.mem_dump;

    case MEM_DUMP_CTRLPT_TYPE:
        return m_diagws.diagw_notif.status.items.mem_dump_ctrlpt;

    default:
        return 0;
    }
}

static bool cccd_written(ble_gatts_evt_write_t const *p_write_evt)
{
    return ((p_write_evt->op == BLE_GATTS_OP_WRITE_REQ) && (p_write_evt->uuid.type == BLE_UUID_TYPE_BLE) &&
            (p_write_evt->uuid.uuid == BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG));
}

bool is_cccd_configured(notif_data_type_t type)
{
    ret_code_t        err_code;
    uint8_t           cccd_val_buf[BLE_CCCD_VALUE_LEN];
    ble_gatts_value_t gatts_value = {.len = BLE_CCCD_VALUE_LEN, .offset = 0, .p_value = cccd_val_buf};

#if APP_MODULE_ENABLED(BLE_SEC)
    // Check if the link is authenticated (meaning at least MITM).
    pm_conn_sec_status_t conn_sec_status;
    err_code = pm_conn_sec_status_get(m_diagws.conn_handle, &conn_sec_status);

    if ((err_code != NRF_SUCCESS) || (conn_sec_status.connected == false) || (conn_sec_status.bonded == false) ||
        (conn_sec_status.encrypted == false))
    {
        return false;
    }
#endif

    err_code = sd_ble_gatts_value_get(m_diagws.conn_handle, m_diagws.char_handle[type].cccd_handle, &gatts_value);

    if (err_code != NRF_SUCCESS)
    {
        return false;
    }

    if (type == MEM_DUMP_CTRLPT_TYPE)
    {
        return ble_srv_is_indication_enabled(cccd_val_buf) && diagw_notif_status_get(type);
    }

    return ble_srv_is_notification_enabled(cccd_val_buf) && diagw_notif_status_get(type);
}

/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the SoftDevice.
 *
 * @param[in] p_diagw     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_connect(ble_diagw_t *p_diagw, ble_evt_t const *p_ble_evt)
{
    p_diagw->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

/**@brief Function for handling the @ref BLE_GAP_EVT_DISCONNECTED event from the SoftDevice.
 *
 * @param[in] p_diagw     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_disconnect(ble_diagw_t *p_diagw, ble_evt_t const *p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_diagw->conn_handle             = BLE_CONN_HANDLE_INVALID;
    p_diagw->diagw_notif.status.byte = 0;
    p_diagw->max_data_len            = BLE_GATT_ATT_MTU_DEFAULT - OVERHEAD_LENGTH;
    mem_dump_on_ble_disconnect_callback(p_diagw);
}

static void on_mem_dump_ctrlpt_write(ble_gatts_evt_write_t const *p_evt_write)
{
    mem_dump_on_ble_cmd_callback((uint8_t *)p_evt_write->data, p_evt_write->len);
}

/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the SoftDevice.
 *
 * @param[in] p_diagw     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(ble_diagw_t *p_diagw, ble_evt_t const *p_ble_evt)
{
    ble_gatts_evt_write_t const *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (cccd_written(p_evt_write))
    {
        for (uint8_t i = 1; i < NOTIF_DATA_TYPE_SIZE; i++)
        {
            if (p_evt_write->handle == p_diagw->char_handle[i].cccd_handle && p_evt_write->len == BLE_CCCD_VALUE_LEN)
            {
                if (i == MEM_DUMP_CTRLPT_TYPE)
                {
                    diagw_notif_status_set(MEM_DUMP_CTRLPT_TYPE, ble_srv_is_indication_enabled(p_evt_write->data) ? 1 : 0);
                }
                else
                {
                    diagw_notif_status_set(i, ble_srv_is_notification_enabled(p_evt_write->data) ? 1 : 0);
                }

                return;
            }
        }
    }

    if ((p_evt_write->handle == p_diagw->rx_handle.value_handle) && (p_diagw->data_handler != NULL))
    {
        ble_diagw_evt_t evt;
        evt.p_diagw               = p_diagw;
        evt.params.rx_data.p_data = p_evt_write->data;
        evt.params.rx_data.length = p_evt_write->len;
        evt.type                  = BLE_DIAGW_EVT_RX_DATA;
        p_diagw->data_handler(&evt);
        return;
    }

    if (p_evt_write->handle == p_diagw->char_handle[MEM_DUMP_CTRLPT_TYPE].value_handle)
    {
        on_mem_dump_ctrlpt_write(p_evt_write);
        return;
    }
}

static void on_rw_auth_req(ble_diagw_t *p_diagw, ble_evt_t const *p_ble_evt)
{
    ret_code_t                            err_code;
    ble_gatts_evt_rw_authorize_request_t  req;
    ble_gatts_rw_authorize_reply_params_t auth_reply;

    req = p_ble_evt->evt.gatts_evt.params.authorize_request;

    if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
    {
        if ((req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE) && (req.request.write.op != BLE_GATTS_OP_PREP_WRITE_REQ) &&
            (req.request.write.op != BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) && (req.request.write.op != BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
        {
            if (req.request.write.handle == p_diagw->char_handle[MEM_DUMP_CTRLPT_TYPE].value_handle)
            {
                // on_mem_dump_ctrlpt_write(&req.request.write);
            }

            // auth_reply.type                     = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
            // auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
            // err_code                            = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle, &auth_reply);
            // APP_ERROR_CHECK(err_code);
        }
        else if (req.type == BLE_GATTS_AUTHORIZE_TYPE_READ)
        {
            if (req.request.read.handle == p_diagw->rx_handle.value_handle)
            {
                data_session_t data_session;
                memset((uint8_t *)&data_session, 0, sizeof(data_session_t));
                data_session_get((const data_session_t *const)&data_session);

                auth_reply.type                    = BLE_GATTS_AUTHORIZE_TYPE_READ;
                auth_reply.params.read.gatt_status = BLE_GATT_STATUS_SUCCESS;
                auth_reply.params.read.update      = 1;
                auth_reply.params.read.len         = sizeof(data_session.timestamp);
                auth_reply.params.read.p_data      = (uint8_t *)&data_session.timestamp;
                auth_reply.params.read.offset      = 0;
                err_code                           = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle, &auth_reply);
                APP_ERROR_CHECK(err_code);
            }
        }
    }
}

static void on_tx_complete(ble_diagw_t *p_diagw, ble_evt_t const *p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);

    if (hvn_tx_busy)
    {
        hvn_tx_busy = false;
    }

    // Notify with empty data that some tx was completed.
    ble_diagw_evt_t evt = {.type = BLE_DIAGW_EVT_TX_RDY, .p_diagw = p_diagw};
    p_diagw->data_handler(&evt);
}

void nrf_ble_diagw_on_gatt_evt(ble_diagw_t *p_diagw, nrf_ble_gatt_evt_t const *p_gatt_evt)
{
    if (p_gatt_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        p_diagw->max_data_len = p_gatt_evt->params.att_mtu_effective - OVERHEAD_LENGTH;
    }
}

static void on_hvc(ble_diagw_t *p_diagw, ble_evt_t const *p_ble_evt)
{
    ble_gatts_evt_hvc_t const *p_hvc = &p_ble_evt->evt.gatts_evt.params.hvc;

    if (p_hvc->handle == p_diagw->char_handle[MEM_DUMP_CTRLPT_TYPE].value_handle)
    {
        mem_dump_on_ble_cmd_ack_callback();
    }
}

void ble_diagw_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    ble_diagw_t *p_diagw = (ble_diagw_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        on_connect(p_diagw, p_ble_evt);
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        on_disconnect(p_diagw, p_ble_evt);
        break;

    case BLE_GATTS_EVT_WRITE:
        on_write(p_diagw, p_ble_evt);
        break;

    case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        on_rw_auth_req(p_diagw, p_ble_evt);
        break;

    case BLE_GATTS_EVT_HVN_TX_COMPLETE:
        on_tx_complete(p_diagw, p_ble_evt);
        break;

    case BLE_GATTS_EVT_HVC:
        on_hvc(p_diagw, p_ble_evt);
        break;

    default:
        // No implementation needed.
        break;
    }
}

/**@brief Function for adding ECG characteristic.
 *
 * @param[in] p_diagw       Nordic UART Service structure.

 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t notif_char_add(notif_data_type_t type, ble_diagw_t *p_diagw, ble_diagw_init_t const *p_diagw_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
#if APP_MODULE_ENABLED(BLE_SEC)
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&cccd_md.write_perm);
#else
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
#endif

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_diagw->uuid_type;
    ble_uuid.uuid = notif_uuid[type];

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = notif_max_len[type];

    return sd_ble_gatts_characteristic_add(p_diagw->service_handle, &char_md, &attr_char_value, &p_diagw->char_handle[type]);
}

static void mem_dump_ctrlpt_char_add(ble_diagw_t *p_diagw)
{
    ret_code_t            err_code;
    ble_add_char_params_t add_params;

    memset(&add_params, 0, sizeof(add_params));

    add_params.uuid                     = BLE_UUID_DIAGW_MEM_DUMP_CTRLPT_CHARACTERISTIC;
    add_params.uuid_type                = p_diagw->uuid_type;
    add_params.max_len                  = MD_CTRLPT_CMD_SIZE;
    add_params.init_len                 = 0;
    add_params.p_init_value             = NULL;
    add_params.char_props.read          = 1;
    add_params.char_props.write         = 1;
    add_params.char_props.write_wo_resp = 1;
    add_params.char_props.indicate      = 1;
#if APP_MODULE_ENABLED(BLE_SEC)
    add_params.read_access       = SEC_JUST_WORKS;
    add_params.write_access      = SEC_JUST_WORKS;
    add_params.cccd_write_access = SEC_JUST_WORKS;
#else
    add_params.read_access       = SEC_OPEN;
    add_params.write_access      = SEC_OPEN;
    add_params.cccd_write_access = SEC_OPEN;
#endif
    add_params.is_var_len = 1;

    err_code = characteristic_add(p_diagw->service_handle, &add_params, &p_diagw->char_handle[MEM_DUMP_CTRLPT_TYPE]);
    APP_ERROR_CHECK(err_code);
}

static void mem_dump_data_char_add(ble_diagw_t *p_diagw)
{
    ret_code_t            err_code;
    ble_add_char_params_t add_params;

    // Add memory dump data characteristic.
    memset(&add_params, 0, sizeof(add_params));

    add_params.uuid              = BLE_UUID_DIAGW_MEM_DUMP_CHARACTERISTIC;
    add_params.uuid_type         = p_diagw->uuid_type;
    add_params.max_len           = NRF_SDH_BLE_GATT_MAX_MTU_SIZE;
    add_params.init_len          = 0;
    add_params.p_init_value      = NULL;
    add_params.char_props.notify = 1;
    add_params.char_props.read   = 1;
#if APP_MODULE_ENABLED(BLE_SEC)
    add_params.cccd_write_access = SEC_JUST_WORKS;
    add_params.read_access       = SEC_JUST_WORKS;
#else
    add_params.cccd_write_access = SEC_OPEN;
    add_params.read_access       = SEC_OPEN;
#endif
    add_params.is_var_len = 1;

    err_code = characteristic_add(p_diagw->service_handle, &add_params, &p_diagw->char_handle[MEM_DUMP_TYPE]);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for adding RX characteristic.
 *
 * @param[in] p_diagw       Nordic UART Service structure.
 * @param[in] p_diagw_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t rx_char_add(ble_diagw_t *p_diagw, const ble_diagw_init_t *p_diagw_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    data_session_t data_session;
    memset((uint8_t *)&data_session, 0, sizeof(data_session_t));
    data_session_get((const data_session_t *const)&data_session);

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write         = 1;
    char_md.char_props.write_wo_resp = 1;
    char_md.char_props.read          = 1;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

    ble_uuid.type = p_diagw->uuid_type;
    ble_uuid.uuid = BLE_UUID_DIAGW_RX_CHARACTERISTIC;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 1;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(data_session.timestamp);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value   = (uint8_t *)&data_session.timestamp;
    attr_char_value.max_len   = BLE_DIAGW_MAX_RX_CHAR_LEN;

    return sd_ble_gatts_characteristic_add(p_diagw->service_handle, &char_md, &attr_char_value, &p_diagw->rx_handle);
}

uint32_t ble_diagw_init(ble_diagw_t *p_diagw, ble_diagw_init_t const *p_diagw_init)
{
    uint32_t      err_code;
    ble_uuid_t    ble_uuid;
    ble_uuid128_t diagw_base_uuid = DIAGW_BASE_UUID;

    VERIFY_PARAM_NOT_NULL(p_diagw);
    VERIFY_PARAM_NOT_NULL(p_diagw_init);

    // Initialize the service structure.
    p_diagw->conn_handle             = BLE_CONN_HANDLE_INVALID;
    p_diagw->diagw_notif.status.byte = 0;
    p_diagw->data_handler            = p_diagw_init->data_handler;
    p_diagw->max_data_len            = BLE_GATT_ATT_MTU_DEFAULT - OVERHEAD_LENGTH;

    // Add a custom base UUID.
    err_code = sd_ble_uuid_vs_add(&diagw_base_uuid, &p_diagw->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_diagw->uuid_type;
    ble_uuid.uuid = BLE_UUID_DIAGW_SERVICE;

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_diagw->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add the ECG Characteristic.
    err_code = notif_char_add(ECG_TYPE, p_diagw, p_diagw_init);
    VERIFY_SUCCESS(err_code);

    // Add the BREATH Characteristic.
    err_code = notif_char_add(BREATH_TYPE, p_diagw, p_diagw_init);
    VERIFY_SUCCESS(err_code);

    // Add the TEMP Characteristic.
    err_code = notif_char_add(TEMP_TYPE, p_diagw, p_diagw_init);
    VERIFY_SUCCESS(err_code);

    // Add the IMP Characteristic.
    err_code = notif_char_add(IMP_TYPE, p_diagw, p_diagw_init);
    VERIFY_SUCCESS(err_code);

    // Add the ACC Characteristic.
    err_code = notif_char_add(ACC_TYPE, p_diagw, p_diagw_init);
    VERIFY_SUCCESS(err_code);

    // Add the memory dump data characteristic.
    mem_dump_data_char_add(p_diagw);

    // Add the memory dump control point characteristic.
    mem_dump_ctrlpt_char_add(p_diagw);

    // Add the RX Characteristic.
    err_code = rx_char_add(p_diagw, p_diagw_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

void diagw_data_handler(ble_diagw_evt_t *p_evt)
{
    if (p_evt->type == BLE_DIAGW_EVT_RX_DATA)
    {
        if (p_evt->params.rx_data.length < 1)
        {
            return;
        }

        // 7 pour Set UIWR
        if ((p_evt->params.rx_data.p_data[0] == cmd_id_device_name_set) && (p_evt->params.rx_data.length >= 5))
        {
            uint8_t serial[4];
            memcpy(serial, (uint32_t *)0x10001080, 4);
            // if (serial[0] == 0xFF)
            // Command to write serial to the User Internal Memory
            //{
            //__irq_disable();
            __sd_nvic_irq_disable();

            // MWU Enable -- This is workaround suggested by Aryan on the Nordic Developer Zone
            NRF_MWU->REGIONENSET =
                ((MWU_REGIONENSET_RGN0WA_Set << MWU_REGIONENSET_RGN0WA_Pos) | (MWU_REGIONENSET_PRGN0WA_Set << MWU_REGIONENSET_PRGN0WA_Pos));

            // Turn on flash write enable and wait until the NVMC is ready:
            NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);
            while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}

            *(uint32_t *)0x10001080 = swap_(*(uint32_t *)(p_evt->params.rx_data.p_data + 1));

            while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}

            // Turn off flash write enable and wait until the NVMC is ready:
            NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);
            while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}

            // MWU Disable -- This is workaround suggested by Aryan on the Nordic Developer Zone
            NRF_MWU->REGIONENCLR = ((MWU_REGIONENCLR_RGN0WA_Clear << MWU_REGIONENCLR_RGN0WA_Pos) |
                                    (MWU_REGIONENCLR_PRGN0WA_Clear << MWU_REGIONENCLR_PRGN0WA_Pos));
            NVIC_SystemReset();
            //}
        }
        else if (p_evt->params.rx_data.p_data[0] == cmd_id_device_shutdwon)
        {
            pm_allow_device_to_sleep();
        }
        else if (p_evt->params.rx_data.p_data[0] == cmd_id_imp_meas_start)
        {
            imp_meas_start();
        }
        else if ((p_evt->params.rx_data.p_data[0] == cmd_id_auto_diag_status_set) && (p_evt->params.rx_data.length >= 2))
        {
#if APP_MODULE_ENABLED(AUTODIAG)
            auto_diag_set_status(p_evt->params.rx_data.p_data[1]);
            auto_diag_system_reset();
#endif
        }
        else
        {
            // Commands to be executed in thread mode of the CPU
            app_ble_cmd_update_callback((ble_cmd_id_t)p_evt->params.rx_data.p_data[0], &p_evt->params.rx_data.p_data[1],
                                        p_evt->params.rx_data.length - 1);
        }
    }
}

static inline bool is_ble_configured(notif_data_type_t type)
{
    return ((m_diagws.conn_handle != BLE_CONN_HANDLE_INVALID) && (m_diagws.max_data_len >= BLE_DIAGW_MAX_RAW_DATA_LEN) &&
            (is_cccd_configured(type)));
}

uint32_t ble_diagw_data_notif_send(notif_data_type_t type, uint8_t *buf, uint16_t len)
{
    ret_code_t             err_code;
    uint16_t               hvx_len;
    ble_gatts_hvx_params_t hvx_params;
    uint8_t                timeout = BLE_TIMEOUT;

    if (len > BLE_DIAGW_MAX_DATA_LEN || (type <= UNKNOWN_TYPE || type >= NOTIF_DATA_TYPE_SIZE))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (!is_ble_configured(type) || !data_session_user_is_connected())
    {
        return NRF_ERROR_INVALID_STATE;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_len           = len;
    hvx_params.handle = m_diagws.char_handle[type].value_handle;
    hvx_params.p_data = buf;
    hvx_params.p_len  = &hvx_len;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    while ((err_code = sd_ble_gatts_hvx(m_diagws.conn_handle, &hvx_params)) != NRF_SUCCESS)
    {
        /* When notification queue is full sd_ble_gatts_hvx will return NRF_ERROR_RESOURCES.
         * We have to wait until hvn_tx_busy is released before sending new data */
        if (err_code == NRF_ERROR_RESOURCES)
        {
            hvn_tx_busy = true;

            while (hvn_tx_busy)
            {
                if (--timeout == 0)
                {
                    INFO("[%s] Timeout", (uint32_t) __func__);
                    return err_code;
                }

                __WFE();
            }
        }
        else
        {
            INFO("[%s] Something went wrong (%u)", (uint32_t) __func__, err_code);
            break;
        }
    }

    if ((err_code == NRF_SUCCESS) && (hvx_len != len))
    {
        INFO("[%s] Live data notification data size error!", (uint32_t) __func__);
        err_code = NRF_ERROR_DATA_SIZE;
    }

    return err_code;
}

ret_code_t ble_diagw_mem_dump_notif_send(uint8_t *buf, uint16_t len)
{
    ret_code_t             err_code;
    uint16_t               hvx_len;
    ble_gatts_hvx_params_t hvx_param;

    if (!is_ble_configured(MEM_DUMP_TYPE) || !data_session_user_is_connected())
    {
        return NRF_ERROR_INVALID_STATE;
    }

    memset(&hvx_param, 0, sizeof(hvx_param));
    hvx_len          = len;
    hvx_param.type   = BLE_GATT_HVX_NOTIFICATION;
    hvx_param.handle = m_diagws.char_handle[MEM_DUMP_TYPE].value_handle;
    hvx_param.p_data = buf;
    hvx_param.p_len  = &hvx_len;
    hvx_param.offset = 0;

    err_code = sd_ble_gatts_hvx(m_diagws.conn_handle, &hvx_param);
    if ((err_code == NRF_SUCCESS) && (hvx_len != len))
    {
        INFO("[%s] Mem. dump notification data size error!", (uint32_t) __func__);
        err_code = NRF_ERROR_DATA_SIZE;
    }

    return err_code;
}

ret_code_t ble_diagw_mem_dump_ctrpt_indic_send(uint8_t *buf, uint16_t len)
{
    ret_code_t             err_code;
    uint16_t               hvx_len;
    ble_gatts_hvx_params_t hvx_param;

    if (!is_ble_configured(MEM_DUMP_CTRLPT_TYPE))
    {
        return NRF_ERROR_INVALID_STATE;
    }

    memset(&hvx_param, 0, sizeof(hvx_param));
    hvx_len          = len;
    hvx_param.type   = BLE_GATT_HVX_INDICATION;
    hvx_param.handle = m_diagws.char_handle[MEM_DUMP_CTRLPT_TYPE].value_handle;
    hvx_param.p_data = buf;
    hvx_param.p_len  = &hvx_len;
    hvx_param.offset = 0;

    err_code = sd_ble_gatts_hvx(m_diagws.conn_handle, &hvx_param);
    if ((err_code == NRF_SUCCESS) && (hvx_len != len))
    {
        INFO("[%s] Mem. dump ctrlpt indication data size error!", (uint32_t) __func__);
        err_code = NRF_ERROR_DATA_SIZE;
    }

    return err_code;
}

uint16_t ble_diagw_max_data_len_get(void)
{
    return m_diagws.max_data_len;
}

ble_diagw_t *ble_diagw_svc_instance_get(void)
{
    return (ble_diagw_t *)&m_diagws;
}