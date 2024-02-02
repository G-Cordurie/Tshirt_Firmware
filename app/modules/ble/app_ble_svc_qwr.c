#include <stdint.h>
#include <string.h>

#include "app_error.h"
#include "nrf_ble_bms.h"
#include "nrf_ble_qwr.h"

#include "app_ble_svc_bms.h"

NRF_BLE_QWR_DEF(m_qwr); // Context for the Queued Write module.

#define MEM_BUFF_SIZE 512

static uint8_t m_qwr_mem[MEM_BUFF_SIZE]; // Write buffer for the Queued Write module.

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static uint16_t qwr_evt_handler(nrf_ble_qwr_t *p_qwr, nrf_ble_qwr_evt_t *p_evt)
{
    nrf_ble_bms_t *p_bms = app_ble_svc_bms_instance_get();
    return nrf_ble_bms_on_qwr_evt(p_bms, p_qwr, p_evt);
}

void app_ble_svc_qwr_init(void)
{
    uint32_t           err_code;
    nrf_ble_qwr_init_t qwr_init;

    // Initialize Queued Write Module
    memset(&qwr_init, 0, sizeof(qwr_init));
    qwr_init.mem_buffer.len   = MEM_BUFF_SIZE;
    qwr_init.mem_buffer.p_mem = m_qwr_mem;
    qwr_init.callback         = qwr_evt_handler;
    qwr_init.error_handler    = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);
}

void app_ble_svc_qwr_conn_handle_assign(uint16_t conn_handle)
{
    uint32_t err_code;

    err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, conn_handle);
    APP_ERROR_CHECK(err_code);
}

nrf_ble_qwr_t *app_ble_svc_qwr_instance_get(void)
{
    return &m_qwr;
}