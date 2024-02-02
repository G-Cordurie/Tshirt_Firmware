#include <stdbool.h>

#include "nrf_drv_rtc.h"

#include "debug.h"
#include "rtc.h"

#define DRV_RTC_MAX_CNT RTC_COUNTER_COUNTER_Msk

static const nrf_drv_rtc_t m_rtc = NRF_DRV_RTC_INSTANCE(2);
static volatile uint64_t   m_base_counter;
static volatile uint64_t   m_stamp64;

static uint64_t get_now(void)
{
    uint64_t now = m_base_counter + nrf_drv_rtc_counter_get(&m_rtc);

    if (now < m_stamp64)
    {
        now += (DRV_RTC_MAX_CNT + 1);
    }

    return now;
}

static void on_overflow_evt(void)
{
    m_base_counter += (DRV_RTC_MAX_CNT + 1);
    INFO("[RTC] > Overflow event");
}

static void on_compare0_evt(void)
{
    m_stamp64 = get_now();
    INFO("[RTC] > CMP0 event");
}

static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    if (int_type == NRFX_RTC_INT_OVERFLOW)
    {
        on_overflow_evt();
    }

    if (int_type == NRFX_RTC_INT_COMPARE0)
    {
        on_compare0_evt();
    }
}

void rtc_init(void)
{
    static bool initialized = false;

    if (!initialized)
    {

        nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;

        config.prescaler = 32;

        ret_code_t err_code = nrf_drv_rtc_init(&m_rtc, &config, rtc_handler);
        APP_ERROR_CHECK(err_code);

        nrf_drv_rtc_tick_disable(&m_rtc);
        nrf_drv_rtc_overflow_enable(&m_rtc, true);
        nrf_drv_rtc_cc_set(&m_rtc, NRF_DRV_RTC_INT_COMPARE0, DRV_RTC_MAX_CNT >> 1, true);
        m_base_counter = 0;
        m_stamp64      = 0;

        initialized = true;
    }
}

void rtc_uninit(void)
{
    nrf_drv_rtc_uninit(&m_rtc);
}

void rtc_start(void)
{
    rtc_stop();
    nrf_drv_rtc_enable(&m_rtc);
}

void rtc_stop(void)
{
    nrf_drv_rtc_disable(&m_rtc);
    nrf_drv_rtc_counter_clear(&m_rtc);
    m_base_counter = 0;
    m_stamp64      = 0;
}

uint64_t rtc_timestamp_get(void)
{
    return get_now();
}