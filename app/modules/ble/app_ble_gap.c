#include <stdint.h>
#include <string.h>

#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_dfu.h"
#include "ble_hci.h"
#include "nrf_log.h"
#include "nrf_stack_guard.h"
#include "peer_manager.h"

#include "app_ble_gap.h"
#include "app_ble_mngr.h"
#include "app_ble_svc_bms.h"
#include "app_config.h"
#include "battery.h"
#include "ble_diagw.h"
#include "boards.h"
#include "data_acq.h"
#include "data_session.h"
#include "debug.h"
#include "pwr_mngr.h"
#include "timer.h"
#include "utils.h"

BLE_ADVERTISING_DEF(m_advertising); // Advertising module instance.

#define APP_BLE_CONN_CFG_TAG           1 // A tag identifying the SoftDevice BLE configuration.

#define DEFAULT_CONN_INTERVAL          (uint16_t)(MSEC_TO_UNITS(15, UNIT_1_25_MS)) // Connection interval to be used at connection params negociation
#define MIN_CONN_INTERVAL              (uint16_t)(MSEC_TO_UNITS(15, UNIT_1_25_MS))  // Minimum acceptable connection interval (15 ms)
#define MAX_CONN_INTERVAL              (uint16_t)(MSEC_TO_UNITS(500, UNIT_1_25_MS)) // Maximum acceptable connection interval (500 ms)
#define CONN_SUP_TIMEOUT               (uint16_t)(MSEC_TO_UNITS(6000, UNIT_10_MS))  // Connection supervisory timeout (6 seconds)
#define SLAVE_LATENCY                  0                                            // Slave latency.

#define APP_ADV_FAST_INTERVAL          MSEC_TO_UNITS(20, UNIT_0_625_MS)     // Fast advertising interval
#define APP_ADV_SLOW_INTERVAL          MSEC_TO_UNITS(152.5F, UNIT_0_625_MS) // Slow advertising interval
#define APP_ADV_FAST_TIMEOUT           3500U                                // Fast advertising timeout (35 seconds)
#define APP_ADV_SLOW_TIMEOUT           8500U                                // Slow advertising timeout (85 seconds)

// Time from initiating event (connect or start of notification) to first * time sd_ble_gap_conn_param_update is called (5 seconds).
#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000)
// Time between each call to sd_ble_gap_conn_param_update after the first * call (30 seconds).
#define NEXT_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(30000)
// Number of attempts before giving up the connection parameter * negotiation.
#define MAX_CONN_PARAMS_UPDATE_COUNT   4

// TX Power Level value. This will be set both in the TX Power service, in the advertising data, and also used to set the radio transmit
// power.
#define TX_POWER_LEVEL                 (4)

ble_conn_config_t m_conn_config = {.att_mtu                  = NRF_SDH_BLE_GATT_MAX_MTU_SIZE,
                                   .conn_interval            = DEFAULT_CONN_INTERVAL,
                                   .conn_evt_len_ext_enabled = true,
                                   .phys.tx_phys             = BLE_GAP_PHY_2MBPS,
                                   .phys.rx_phys             = BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS};

static char       device_name[12];
static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_DIAGW_SERVICE, DIAGW_SERVICE_UUID_TYPE}};

// Connection parameters requested for connection.
static ble_gap_conn_params_t m_conn_param = {
    .min_conn_interval = MIN_CONN_INTERVAL, // Minimum connection interval.
    .max_conn_interval = MAX_CONN_INTERVAL, // Maximum connection interval.
    .slave_latency     = SLAVE_LATENCY,     // Slave latency.
    .conn_sup_timeout  = CONN_SUP_TIMEOUT   // Supervisory timeout.
};

static volatile ble_adv_status_t m_ble_advertising = ble_adv_status_idle;

char const *phy_str(ble_gap_phys_t phys)
{
    static char const *str[] = {"1 Mbps", "2 Mbps", "Coded", "Unknown"};

    switch (phys.tx_phys)
    {
    case BLE_GAP_PHY_1MBPS:
        return str[0];

    case BLE_GAP_PHY_2MBPS:
    case BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS:
    case BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS | BLE_GAP_PHY_CODED:
        return str[1];

    case BLE_GAP_PHY_CODED:
        return str[2];

    default:
        return str[3];
    }
}

/**
 * C++ version 0.4 char* style "itoa":
 * Written by Lukás Chmela
 * Released under GPLv3.
 */
char *itoa(int value, char *result, int base)
{
    // check that the base if valid
    if (base < 2 || base > 36)
    {
        *result = '\0';
        return result;
    }

    char *ptr = result, *ptr1 = result, tmp_char;
    int   tmp_value;

    do
    {
        tmp_value = value;
        value /= base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrst"
                 "uvwxyz"[35 + (tmp_value - value * base)];
    } while (value);

    // Apply negative sign
    if (tmp_value < 0)
        *ptr++ = '-';
    *ptr-- = '\0';
    while (ptr1 < ptr)
    {
        tmp_char = *ptr;
        *ptr--   = *ptr1;
        *ptr1++  = tmp_char;
    }
    return result;
}

void device_name_base36_set(void)
{
    uint32_t serial;

    strcpy(device_name, BLE_DEVICE_NAME);

    serial = *(uint32_t *)0x10001080;

    if (serial < 36)
    {
        itoa(serial, &device_name[9], 36);
    }
    else if (serial < 1296)
    {
        itoa(serial, &device_name[8], 36);
    }
    else if (serial < 46656)
    {
        itoa(serial, &device_name[7], 36);
    }
    else
    {
        itoa(serial, &device_name[6], 36);
    }
}

void device_name_base10_set(uint32_t value)
{
    memset(device_name, 0, sizeof(device_name));
    strcpy(device_name, BLE_DEVICE_NAME);
    sprintf(&device_name[6], "%u", (unsigned int)value);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{

    switch (ble_adv_evt)
    {

    case BLE_ADV_EVT_IDLE:
        INFO("No connectable advertising is ongoing");
        advertising_status_set(ble_adv_status_idle);
        pm_allow_device_to_sleep();
        break;

    case BLE_ADV_EVT_DIRECTED:
        INFO("Directed advertising (low duty cycle) has started!");
        break;

    case BLE_ADV_EVT_DIRECTED_HIGH_DUTY:
        INFO("Direct advertising mode has started!");
        break;

    case BLE_ADV_EVT_FAST:
        advertising_status_set(ble_adv_status_ongoing);
        INFO("Fast advertising mode has started!");
        break;

    case BLE_ADV_EVT_SLOW:
        advertising_status_set(ble_adv_status_ongoing);
        INFO("Slow advertising mode has started!");
        break;

    case BLE_ADV_EVT_FAST_WHITELIST:
        INFO("Fast advertising mode using the whitelist has started!");
        break;

    case BLE_ADV_EVT_SLOW_WHITELIST:
        INFO("Slow advertising mode using the whitelist has started!");
        break;

    case BLE_ADV_EVT_WHITELIST_REQUEST:
        INFO("Request a whitelist from the main application!");
        break;

    case BLE_ADV_EVT_PEER_ADDR_REQUEST:
        INFO("Request a peer address from the main application!");
        break;

    default:
        break;
    }
}

/**@brief Function for handling advertising errors.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void ble_advertising_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
    case BLE_CONN_PARAMS_EVT_FAILED:
        INFO("Connection params update failed!");
        // ret_code_t err_code;
        // uint16_t conn_handle = ble_conn_handle_get();
        // err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        // APP_ERROR_CHECK(err_code);
        break;

    case BLE_CONN_PARAMS_EVT_SUCCEEDED:
        INFO("Connection params update succeeded.");
        break;

    default:
        break;
    }
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    INFO("Function: %s, err: 0x%x", (uint32_t) __func__, nrf_error);
    // APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)device_name, strlen(device_name));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_ppcp_set(&m_conn_param);
    APP_ERROR_CHECK(err_code);
}

void advertising_config_get(ble_adv_modes_config_t *p_config)
{
    memset(p_config, 0, sizeof(ble_adv_modes_config_t));

    // Adv modes configurations
    p_config->ble_adv_on_disconnect_disabled = false;
    p_config->ble_adv_whitelist_enabled      = false;

    // Adv mode: Fast
    p_config->ble_adv_fast_enabled  = true;
    p_config->ble_adv_fast_interval = APP_ADV_FAST_INTERVAL; // 20 ms
    p_config->ble_adv_fast_timeout  = APP_ADV_FAST_TIMEOUT;  // 35 s

    // Adv mode: Slow
    p_config->ble_adv_slow_enabled  = true;
    p_config->ble_adv_slow_interval = APP_ADV_SLOW_INTERVAL;                 // 152.5 ms
    p_config->ble_adv_slow_timeout  = BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED; // No timeout
}

void advertising_modes_config_set(ble_adv_modes_config_t *p_config)
{
    if (p_config)
    {
        ble_advertising_modes_config_set(&m_advertising, p_config);
    }
}

/**@brief Function for initializing the Advertising functionality.
 */
#if !APP_MODULE_ENABLED(AUTODIAG)
void advertising_init(void)
{
    ble_advertising_init_t init;

    // Adv packet data (Flags + 128bit UUID)
    memset(&init, 0, sizeof(init));
    init.advdata.include_appearance      = false;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    // Scan response packet data (Device name)
    init.srdata.name_type = BLE_ADVDATA_FULL_NAME;

    advertising_config_get(&init.config);
    init.config.ble_adv_on_disconnect_disabled = true;
    init.config.ble_adv_slow_timeout           = APP_ADV_SLOW_TIMEOUT;

    init.evt_handler   = on_adv_evt;
    init.error_handler = ble_advertising_error_handler;

    ret_code_t err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}
#else // AUTO DIAG Adv. config.
void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    // Adv packet data (Flags + Device name + Manufacturer specific data)
    memset(&init, 0, sizeof(init));
    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    // Scan response packet data (128bit UUID)
    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    advertising_config_get(&init.config);
    init.config.ble_adv_on_disconnect_disabled = true;
    init.config.ble_adv_slow_timeout           = APP_ADV_SLOW_TIMEOUT;

    init.evt_handler   = on_adv_evt;
    init.error_handler = ble_advertising_error_handler;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}
#endif

/**@brief Function for starting advertising.
 */
ret_code_t advertising_start(bool erase_bonds)
{
    ret_code_t err_code;

    if (erase_bonds == true)
    {
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
        err_code = delete_all_bonds();
    }
    else
    {
        err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    }

    if (err_code != NRF_SUCCESS)
    {
        INFO("[%s] error (%u)", (uint32_t) __func__, err_code);
    }

    return err_code;
}

/**@brief Function for initializing the Connection Parameters module.
 */
void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    m_conn_param.min_conn_interval = m_conn_config.conn_interval;
    m_conn_param.max_conn_interval = m_conn_config.conn_interval;

    cp_init.p_conn_params                  = &m_conn_param;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

static void disconnect(uint16_t conn_handle, void *p_context)
{
    UNUSED_PARAMETER(p_context);

    ret_code_t err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_SUCCESS)
    {
        INFO("Failed to disconnect connection. Connection handle: %d Error: %d", conn_handle, err_code);
    }
    else
    {
        INFO("Disconnected connection handle %d", conn_handle);
    }
}

// YOUR_JOB: Update this code if you want to do anything given a DFU event (optional).
/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */
void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
    case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
    {
        INFO("Device is preparing to enter bootloader mode.");

        data_session_timeout_tmr_stop();

        // Prevent device from advertising on disconnect.
        ble_adv_modes_config_t config;
        advertising_config_get(&config);
        config.ble_adv_on_disconnect_disabled = true;
        advertising_modes_config_set(&config);

        // Disconnect all other bonded devices that currently are connected.
        // This is required to receive a service changed indication
        // on bootup after a successful (or aborted) Device Firmware Update.
        uint32_t conn_count = ble_conn_state_for_each_connected(disconnect, NULL);
        UNUSED_VARIABLE(conn_count);
        INFO("Disconnected %d links.", conn_count);

        ret_code_t err_code = nrf_stack_guard_deinit();
        APP_ERROR_CHECK(err_code);
        break;
    }

    case BLE_DFU_EVT_BOOTLOADER_ENTER:
        // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
        //           by delaying reset by reporting false in app_shutdown_handler
        INFO("Device will enter bootloader mode.");
        break;

    case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
        INFO("Request to enter bootloader mode failed asynchroneously.");
        // YOUR_JOB: Take corrective measures to resolve the issue
        //           like calling APP_ERROR_CHECK to reset the device.
        break;

    case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
        INFO("Request to send a response to client failed.");
        // YOUR_JOB: Take corrective measures to resolve the issue
        //           like calling APP_ERROR_CHECK to reset the device.
        APP_ERROR_CHECK(false);
        sd_nvic_SystemReset();
        break;

    default:
        INFO("Unknown event from ble_dfu_buttonless.");
        break;
    }
}

void tx_power_set(void)
{
    // Set the maximum power: 4dBm
    // Not much difference from 0dBm but it might help

    ret_code_t err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, TX_POWER_LEVEL);
    APP_ERROR_CHECK(err_code);
}

void advertising_stop(void)
{
    ret_code_t err_code = sd_ble_gap_adv_stop(m_advertising.adv_handle);
    ret_code_verify(err_code);
    advertising_status_set(ble_adv_status_idle);
    INFO("[APP_BLE_GAP] > Advertising stopped");
}

void ble_disconnect(void)
{
    if (is_ble_connected())
    {
        ret_code_t err_code = sd_ble_gap_disconnect(ble_conn_handle_get(), BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE)
        {
            INFO("Failed to disconnect connection. Error: %d !", err_code);
        }
    }
}

void conn_evt_len_ext_set(bool status)
{
    ret_code_t err_code;
    ble_opt_t  opt;

    memset(&opt, 0x00, sizeof(opt));
    opt.common_opt.conn_evt_ext.enable = status ? 1 : 0;

    err_code = sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt);
    APP_ERROR_CHECK(err_code);
}

void preferred_phy_set(ble_gap_phys_t *p_phy)
{
    memcpy(&m_conn_config.phys, p_phy, sizeof(ble_gap_phys_t));
}

ble_adv_status_t advertising_status_get(void)
{
    return m_ble_advertising;
}

void advertising_status_set(ble_adv_status_t status)
{
    m_ble_advertising = status;
}
