#include <stdarg.h>
#include <stdint.h>

#include "arm_math.h"

#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "app_ble_mngr.h"
#include "app_ble_svc_nus.h"
#include "battery.h"
#include "battery_chrgr.h"
#include "battery_soc.h"
#include "debug.h"
#include "rtc.h"
#include "timer.h"

#if APP_MODULE_ENABLED(APP_BLE_NUS)
APP_TIMER_DEF(m_log_tmr_id);

#define LOG_PERIOD_MS    1000U
#define LOG_TMR_INTERVAL APP_TIMER_TICKS(LOG_PERIOD_MS)

static void log_tmr_handler(void *p_context)
{
    // Battery
    float64_t battery_volt     = battery_voltage_get();
    float64_t battery_soc      = battery_percent_compute(CHRGR_DISCONNECTED_STATE, battery_volt);
    uint32_t  battery_volt_dec = battery_volt * 1000;
    log_ble("[Battery] " NRF_LOG_FLOAT_MARKER " V (%u mV) (" NRF_LOG_FLOAT_MARKER " %%)", NRF_LOG_FLOAT(battery_volt), battery_volt_dec,
            NRF_LOG_FLOAT(battery_soc));
}

void log_ble(__const char *__restrict __format, ...)
{
    // TODO: Check if there is a DFU in progress...
    static uint8_t str_buffer[BLE_NUS_MAX_DATA_LEN];
    va_list        args;
    va_start(args, __format);
    int32_t str_max_len = ble_nus_max_date_len_get();
    int32_t str_len     = vsnprintf((char *)str_buffer, str_max_len, __format, args);
    va_end(args);

    // Log errors
    if (str_len > str_max_len)
    {
        INFO("[BLE LOG] too long string (len: %d > max len: %d) !\n", str_len, str_max_len);
    }
    else if (str_len < 0)
    {
        INFO("[BLE LOG] Somthing went wrong (len: %d > max len: %d) !\n", str_len, str_max_len);
    }

    // Send log over BLE
    if (str_len > 0)
    {
        ret_code_t err_code;
        do
        {
            uint16_t len = (uint16_t)strlen((char *)str_buffer);
            err_code     = ble_nus_data_send(ble_nus_svc_instance_get(), str_buffer, &len, ble_conn_handle_get());
            if ((err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) && (err_code != NRF_ERROR_NOT_FOUND))
            {
                // APP_ERROR_CHECK(err_code);
            }
        } while (err_code == NRF_ERROR_RESOURCES);
    }
}

void log_tmr_start(void)
{
    ret_code_t err_code = app_timer_start(m_log_tmr_id, LOG_TMR_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

void log_tmr_stop(void)
{
    ret_code_t err_code = app_timer_stop(m_log_tmr_id);
    APP_ERROR_CHECK(err_code);
}
#endif

#if DEBUG
static uint32_t rtc2_cnt_get(void)
{
    return (uint32_t)rtc_timestamp_get();
}
#endif

void log_init(void)
{
    uint32_t err_code;
#if DEBUG
    err_code = NRF_LOG_INIT(rtc2_cnt_get);
#else
    err_code = NRF_LOG_INIT(NULL);
#endif

    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

#if APP_MODULE_ENABLED(APP_BLE_NUS)
    err_code = app_timer_create(&m_log_tmr_id, APP_TIMER_MODE_REPEATED, log_tmr_handler);
    APP_ERROR_CHECK(err_code);
#endif
}