#include <stdbool.h>

#include "arm_math.h"

#include "nordic_common.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpio.h"
#include "sdk_errors.h"

#include "app_ble_gap.h"
#include "app_ble_mngr.h"
#include "app_ble_svc_bas.h"
#include "battery.h"
#include "battery_chrgr.h"
#include "battery_soc.h"
#include "boards.h"
#include "debug.h"
#include "pwr_mngr.h"
#include "timer.h"
#include "utils.h"

static volatile chrgr_state_t m_chrgr_state = CHRGR_UNKNOWN_STATE;

#if (HW_TYPE == HW_TYPE_0706)
APP_TIMER_DEF(m_chrgr_delay_tmr_id);
APP_TIMER_DEF(m_chrgr_debounce_tmr_id);
APP_TIMER_DEF(m_chrgr_detect_tmr_id);

#define TMR_WAIT_DELAY_MS                   4000U
#define SOC_UPDATE_DELAY_MS                 30000U
#define CHRG_COMPLETE_PERIOD_MS             ((uint32_t)(120 * 60 * 1000))

#define CHRGR_AUTO_RECONN_RETRY_INTERVAL_MS 500U
#define CHRGR_DEFAULT_DEBOUNCE_TIME_MS      500U
#define CHRGR_DISC_DEBOUNCE_TIME_MS         (CHRGR_AUTO_RECONN_RETRY_INTERVAL_MS / 2U)
#define CHRGR_DISC_TIMEOUT_MS               (2U * (CHRGR_AUTO_RECONN_RETRY_INTERVAL_MS + CHRGR_DISC_DEBOUNCE_TIME_MS))

#define CHRGR_CONN_DELAY_MS                 2000U
#define CHRGR_CONN_DEBOUNCE_TIME_MS         (CHRGR_CONN_DELAY_MS / 2U)
#define CHRGR_CONN_TIMEOUT_MS               (2U * (CHRGR_CONN_DELAY_MS + CHRGR_CONN_DEBOUNCE_TIME_MS))

typedef enum tsCtrl_state_tag
{
    tsCtrl_set_state,
    tsCtrl_highZ_state,
    tsCtrl_clear_state,
    tsCtrl_error_state,
} tsCtrl_state_t;

static volatile bool                 m_delay_tmr_timeout      = false;
static volatile bool                 m_chrgr_detect_timeout   = false;
static volatile uint32_t             m_chrgr_debounce_time_ms = CHRGR_DEFAULT_DEBOUNCE_TIME_MS;
static volatile battery_chrg_state_t m_next_state             = CHRG_IDLE_STATE;

static inline void tsCtrl_pin_highZ(void)
{
    nrf_gpio_cfg_input(SCL_SPARE, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_input_disconnect(SCL_SPARE);
    // INFO("[BATTERY_CHRGR] > tsCtrl_pin_highZ");
}

static inline void tsCtrl_pin_set(void)
{
    nrf_gpio_pin_set(SCL_SPARE);
    nrf_gpio_cfg_output(SCL_SPARE);
    // INFO("[BATTERY_CHRGR] > tsCtrl_pin_set");
}

static inline void tsCtrl_pin_clear(void)
{
    nrf_gpio_pin_clear(SCL_SPARE);
    nrf_gpio_cfg_output(SCL_SPARE);
    // INFO("[BATTERY_CHRGR] > tsCtrl_pin_clear");
}

static void chrgr_delay_tmr_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);
    m_delay_tmr_timeout = true;
}

static inline void chrgr_delay_tmr_start(uint32_t timeout_ms)
{
    m_delay_tmr_timeout = false;
    ret_code_t err_code = app_timer_start(m_chrgr_delay_tmr_id, APP_TIMER_TICKS(timeout_ms), NULL);
    APP_ERROR_CHECK(err_code);
}

static inline void chrgr_delay_tmr_stop(void)
{
    ret_code_t err_code = app_timer_stop(m_chrgr_delay_tmr_id);
    APP_ERROR_CHECK(err_code);
    m_delay_tmr_timeout = false;
}

static void chrgr_debounce_tmr_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);

    m_chrgr_state = nrf_drv_gpiote_in_is_set(NCHG) ? CHRGR_DISCONNECTED_STATE : CHRGR_CONNECTED_STATE;

    INFO("[BATTERY_CHRGR] > Charger %s (" NRF_LOG_FLOAT_MARKER " Volts)\n",
         m_chrgr_state == CHRGR_CONNECTED_STATE ? "CONNECTED" : "DISCONNECTED", NRF_LOG_FLOAT(battery_voltage_get()));
}

static inline void chrgr_debounce_tmr_start(void)
{
    ret_code_t err_code = app_timer_start(m_chrgr_debounce_tmr_id, APP_TIMER_TICKS(m_chrgr_debounce_time_ms), NULL);
    APP_ERROR_CHECK(err_code);
}

static inline void chrgr_debounce_tmr_stop(void)
{
    ret_code_t err_code = app_timer_stop(m_chrgr_debounce_tmr_id);
    APP_ERROR_CHECK(err_code);
}

static void chrgr_detect_tmr_handler(void *p_context)
{
    m_chrgr_detect_timeout = true;
}

static inline void chrgr_detect_tmr_start(uint32_t timeout_ms)
{
    m_chrgr_detect_timeout = false;
    ret_code_t err_code    = app_timer_start(m_chrgr_detect_tmr_id, APP_TIMER_TICKS(timeout_ms), NULL);
    APP_ERROR_CHECK(err_code);
}

static inline void chrgr_detect_tmr_stop(void)
{
    ret_code_t err_code = app_timer_stop(m_chrgr_detect_tmr_id);
    APP_ERROR_CHECK(err_code);
    m_chrgr_detect_timeout = false;
}

static void chrgr_pin_state_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    UNUSED_PARAMETER(pin);
    UNUSED_PARAMETER(action);

    m_chrgr_state = CHRGR_UNKNOWN_STATE;

    // INFO("[BATTERY_CHRGR] > Pin state %u CHRGR_UNKNOWN_STATE (" NRF_LOG_FLOAT_MARKER " Volts)", nrf_drv_gpiote_in_is_set(NCHG),
    //      NRF_LOG_FLOAT(battery_voltage_get()));

    chrgr_debounce_tmr_stop();
    chrgr_debounce_tmr_start();
}

// static inline void state_log(battery_chrg_state_t state, chrgr_state_t chrgr_state, float64_t battery_volt, float64_t battery_soc)
// {
//     INFO("[BATTERY_CHRGR] > State: %u, Charger %s ", state,
//          (chrgr_state == CHRGR_CONNECTED_STATE) ? "CONNECTED" : ((chrgr_state == CHRGR_DISCONNECTED_STATE) ? "DISCONNECTED" :
//          "UNKNOWN"));
//     INFO("(" NRF_LOG_FLOAT_MARKER " %%, " NRF_LOG_FLOAT_MARKER " Volts)", NRF_LOG_FLOAT(battery_soc), NRF_LOG_FLOAT(battery_volt));
// }

static battery_chrg_state_t idle_state_hanlder(void)
{
    static bool initilized = false;

    if (!initilized)
    {
        INFO("[BATTERY_CHRGR] > IDLE_STATE");
        m_chrgr_debounce_time_ms = CHRGR_DEFAULT_DEBOUNCE_TIME_MS;
        tsCtrl_pin_highZ();
        chrgr_delay_tmr_stop();
        chrgr_delay_tmr_start(SOC_UPDATE_DELAY_MS);
        initilized = true;
    }

    if (m_chrgr_state == CHRGR_CONNECTED_STATE)
    {
        initilized = false;
        return CHRG_WAIT_STATE;
    }

    if (m_chrgr_state == CHRGR_DISCONNECTED_STATE)
    {
        float64_t battery_volt = battery_voltage_get();
        float64_t battery_soc  = battery_percent_compute(CHRGR_DISCONNECTED_STATE, battery_volt);

        // state_log(CHRG_IDLE_STATE, CHRGR_DISCONNECTED_STATE, battery_volt, battery_soc);

        if (m_delay_tmr_timeout)
        {
            if (isless(battery_soc, (float64_t)battery_level_last_get()))
            {
                UNUSED_RETURN_VALUE(battery_level_set((uint8_t)round(battery_soc)));
            }

            chrgr_delay_tmr_start(SOC_UPDATE_DELAY_MS);
        }

        if (isless(battery_soc, BATTERY_SHUTDOWN_THRESHOLD))
        {
            INFO("[BATTERY_CHRGR] > Battery is discharged: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(battery_soc));
            pm_allow_device_to_sleep();
        }
    }

    return CHRG_IDLE_STATE;
}

static battery_chrg_state_t wait_state_hanlder(void)
{
    static bool initilized = false;

    if (!initilized)
    {
        INFO("[BATTERY_CHRGR] > WAIT_STATE");
        chrgr_delay_tmr_stop();
        chrgr_delay_tmr_start(TMR_WAIT_DELAY_MS);
        initilized = true;
    }

    if (m_chrgr_state == CHRGR_DISCONNECTED_STATE)
    {
        chrgr_delay_tmr_stop();
        initilized = false;
        return CHRG_IDLE_STATE;
    }

    if (m_chrgr_state == CHRGR_CONNECTED_STATE && m_delay_tmr_timeout)
    {
        initilized = false;
        return CHRG_ONGOING_STATE;
    }

    return CHRG_WAIT_STATE;
}

static battery_chrg_state_t chrg_state_hanlder(void)
{
    static bool initilized = false;

    if (!initilized)
    {
        INFO("[BATTERY_CHRGR] > CHRG_ONGOING_STATE");
        tsCtrl_pin_highZ();
        initilized = true;
    }

    if (m_chrgr_state == CHRGR_DISCONNECTED_STATE)
    {
        initilized = false;
        return CHRG_IDLE_STATE;
    }

    if (m_chrgr_state == CHRGR_CONNECTED_STATE)
    {
        float64_t battery_volt = battery_voltage_get();
        float64_t battery_soc  = battery_percent_compute(CHRGR_CONNECTED_STATE, battery_volt);

        // state_log(CHRG_ONGOING_STATE, CHRGR_CONNECTED_STATE, battery_volt, battery_soc);

        if (isgreaterequal(battery_soc, BATTERY_CHARGED_THRESHOLD))
        {
            initilized = false;
            UNUSED_RETURN_VALUE(battery_level_set(100U));
            return CHRG_COMPLETE_STATE;
        }
    }

    return CHRG_ONGOING_STATE;
}

static battery_chrg_state_t chrg_complete_state_hanlder(void)
{
    static bool           initilized              = false;
    static bool           tsCtrl_state_initilized = false;
    static tsCtrl_state_t tsCtrl_next_state       = tsCtrl_set_state;

    if (!initilized)
    {
        INFO("[BATTERY_CHRGR] > CHRG_COMPLETET_STATE");
        chrgr_detect_tmr_stop();
        chrgr_delay_tmr_stop();
        chrgr_delay_tmr_start(CHRG_COMPLETE_PERIOD_MS);
        m_chrgr_debounce_time_ms = CHRGR_DISC_DEBOUNCE_TIME_MS;
        tsCtrl_state_initilized  = false;
        tsCtrl_next_state        = m_chrgr_state == CHRGR_CONNECTED_STATE ? tsCtrl_set_state : tsCtrl_error_state;
        initilized               = true;
    }

    if (m_chrgr_detect_timeout)
    {
        tsCtrl_next_state = tsCtrl_error_state;
    }

    switch (tsCtrl_next_state)
    {
    case tsCtrl_set_state:
    {
        if (!tsCtrl_state_initilized)
        {
            m_chrgr_debounce_time_ms = CHRGR_DISC_DEBOUNCE_TIME_MS;
            tsCtrl_pin_set();
            chrgr_detect_tmr_start(CHRGR_DISC_TIMEOUT_MS);
            tsCtrl_state_initilized = true;
        }

        if (m_chrgr_state == CHRGR_DISCONNECTED_STATE)
        {
            chrgr_detect_tmr_stop();
            tsCtrl_state_initilized = false;
            tsCtrl_next_state       = tsCtrl_highZ_state;
        }
    }
    break;

    case tsCtrl_highZ_state:
    {
        if (!tsCtrl_state_initilized)
        {
            m_chrgr_debounce_time_ms = CHRGR_CONN_DEBOUNCE_TIME_MS;
            tsCtrl_pin_highZ();
            chrgr_detect_tmr_start(CHRGR_CONN_TIMEOUT_MS);
            tsCtrl_state_initilized = true;
        }

        if (m_chrgr_state == CHRGR_CONNECTED_STATE)
        {
            chrgr_detect_tmr_stop();
            tsCtrl_state_initilized = false;

            if (m_delay_tmr_timeout)
            {
                tsCtrl_next_state = tsCtrl_clear_state;
                break;
            }

            tsCtrl_next_state = tsCtrl_set_state;
        }
    }
    break;

    case tsCtrl_clear_state:
    {
        if (!tsCtrl_state_initilized)
        {
            m_chrgr_debounce_time_ms = CHRGR_DISC_DEBOUNCE_TIME_MS;
            tsCtrl_pin_clear();
            chrgr_detect_tmr_start(CHRGR_DISC_TIMEOUT_MS);
            tsCtrl_state_initilized = true;
        }

        if (m_chrgr_state == CHRGR_DISCONNECTED_STATE)
        {
            chrgr_detect_tmr_stop();
            initilized = false;
            return CHRG_IDLE_STATE;
        }
    }
    break;

    case tsCtrl_error_state:
    {
        INFO("[BATTERY_CHRGR] > CHRG_COMPLETET_STATE: TsCtrl Error!\n");
        chrgr_delay_tmr_stop();
        chrgr_detect_tmr_stop();
        initilized = false;
        return CHRG_IDLE_STATE;
    }

    default:
        break;
    }

    return CHRG_COMPLETE_STATE;
}
#endif

void battery_chrgr_enable(void)
{
#if (HW_TYPE == HW_TYPE_0706)
    tsCtrl_pin_highZ();
#endif
}

chrgr_state_t battery_chrgr_state_get(void)
{
    return m_chrgr_state;
}

bool battery_is_charging(void)
{
    return m_next_state != CHRG_IDLE_STATE;
}

void battery_chrgr_init(void)
{
#if (HW_TYPE == HW_TYPE_0706)
    ret_code_t err_code = app_timer_create(&m_chrgr_delay_tmr_id, APP_TIMER_MODE_SINGLE_SHOT, chrgr_delay_tmr_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_chrgr_debounce_tmr_id, APP_TIMER_MODE_SINGLE_SHOT, chrgr_debounce_tmr_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_chrgr_detect_tmr_id, APP_TIMER_MODE_SINGLE_SHOT, chrgr_detect_tmr_handler);
    APP_ERROR_CHECK(err_code);

    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

    nrf_drv_gpiote_in_config_t cfg = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    cfg.pull                       = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(NCHG, &cfg, chrgr_pin_state_handler);
    APP_ERROR_CHECK(err_code);

    m_chrgr_state = nrf_drv_gpiote_in_is_set(NCHG) ? CHRGR_DISCONNECTED_STATE : CHRGR_CONNECTED_STATE;

    tsCtrl_pin_highZ();

    nrf_drv_gpiote_in_event_enable(NCHG, true);

#elif (HW_TYPE == HW_TYPE_0705)
    nrf_gpio_cfg_input(SCL_SPARE, NRF_GPIO_PIN_PULLUP);
    m_chrgr_state = nrf_gpio_pin_read(SCL_SPARE) ? CHRGR_DISCONNECTED_STATE : CHRGR_CONNECTED_STATE;
#endif
}

void battery_chrgr_task(void)
{
#if (HW_TYPE == HW_TYPE_0706)
    if (battery_voltage_meas_started())
    {
        switch (m_next_state)
        {
        case CHRG_IDLE_STATE:
            m_next_state = idle_state_hanlder();
            break;

        case CHRG_WAIT_STATE:
            m_next_state = wait_state_hanlder();
            break;

        case CHRG_ONGOING_STATE:
            m_next_state = chrg_state_hanlder();
            break;

        case CHRG_COMPLETE_STATE:
            m_next_state = chrg_complete_state_hanlder();
            break;

        default:
            break;
        }
    }
#elif (HW_TYPE == HW_TYPE_0705)
    m_chrgr_state = nrf_gpio_pin_read(SCL_SPARE) ? CHRGR_DISCONNECTED_STATE : CHRGR_CONNECTED_STATE;
#endif
}