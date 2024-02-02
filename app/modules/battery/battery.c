#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "arm_math.h"

#include "nrf_delay.h"
#include "nrf_gpio.h"

#include "adc.h"
#include "app_ble_svc_bas.h"
#include "battery.h"
#include "battery_chrgr.h"
#include "battery_soc.h"
#include "boards.h"
#include "debug.h"
#include "pwr_mngr.h"
#include "temp.h"
#include "utils.h"

#define BATTERY_VOLT_5V                 ((float64_t)5.0) // Initial voltage (to distinguish from real values)
#define BATTERY_VOLT_MEAS_PERIOD_MS     1500U
#define BATTERY_SAFE_BOOT_MAX_RETRY_NBR 10U

static volatile float64_t m_battery_voltage = BATTERY_VOLT_5V;

void battery_switch_enable(void)
{
    nrf_gpio_pin_set(EN_SW);
    nrf_gpio_cfg_output(EN_SW);
}

void battery_switch_disable(void)
{
    nrf_gpio_pin_clear(EN_SW);
    nrf_gpio_cfg_output(EN_SW);
}

static float64_t battery_adc_to_calib_volt_convert(float64_t adc_val)
{
    float64_t battery_volt;
    float64_t vcc        = temp_vcc_get();
    float64_t calib_coef = temp_calib_coef_get();

    battery_volt = calib_coef * 2048.0;

    if (isfinite(battery_volt) && isgreater(battery_volt, 0.0))
    {
        battery_volt = (adc_val * vcc) / battery_volt;
        if (isfinite(battery_volt))
        {
            return battery_volt;
        }
    }

    return 0.0;
}

void battery_voltage_update(float64_t adc_val)
{
    m_battery_voltage = battery_adc_to_calib_volt_convert(adc_val);
    // INFO("m_battery_voltage: " NRF_LOG_FLOAT_MARKER "", NRF_LOG_FLOAT(m_battery_voltage));
}

float64_t battery_voltage_get(void)
{
    return m_battery_voltage;
}

bool battery_voltage_meas_started(void)
{
    return isless(m_battery_voltage, BATTERY_VOLT_5V) ? true : false;
}

float64_t battery_adc_to_raw_volt_convert(float64_t adc_val)
{
    float64_t battery_volt;

    battery_volt = adc_val * 0.001464844; // Vref = 3V; Vin = adc_val * (Vref/4096) = adc_val * (0.000732422); Vbatt = 2 * Vin
    if (isfinite(battery_volt))
    {
        return battery_volt;
    }

    return 0.0;
}

static void battery_adc_callback(nrf_drv_saadc_evt_t const *p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        if (p_event->data.done.size == SAADC_BATTERY_BUF_LEN)
        {
            nrf_saadc_value_t adc_buffer[SAADC_BATTERY_CH_SAMPLES] = {0};

            ret_code_t err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_BATTERY_BUF_LEN);
            APP_ERROR_CHECK(err_code);

            adc_channel_buf_cpy(adc_buffer, p_event->data.done.p_buffer, SAADC_BATTERY_CH_SAMPLES, SAADC_CH_0, SAADC_BATTERY_CH_NBR);
            battery_voltage_update(adc_buffer_avg((nrf_saadc_value_t const *const)adc_buffer, SAADC_BATTERY_CH_SAMPLES));
        }
    }
}

ret_code_t battery_safe_boot_check(void)
{
    adc_battery_init(battery_adc_callback);

    chrgr_state_t chrgr_state;

    do
    {
        nrf_delay_ms(BATTERY_VOLT_MEAS_PERIOD_MS); // TODO: Send µC to low power mode
    } while ((chrgr_state = battery_chrgr_state_get()) == CHRGR_UNKNOWN_STATE);

    float64_t battery_soc = battery_percent_compute(chrgr_state, m_battery_voltage);

    if (isless(battery_soc, BATTERY_POWERUP_THRESHOLD))
    {
        INFO("[Battery] > Battery is discharged: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(battery_soc));
        return NRF_ERROR_INVALID_STATE;
    }

    INFO("[Battery] > Battery SOC: " NRF_LOG_FLOAT_MARKER "%%", NRF_LOG_FLOAT(battery_soc));

    UNUSED_RETURN_VALUE(battery_level_set((uint8_t)round(battery_soc)));

    return NRF_SUCCESS;
}

void battery_meas_start(void)
{
    adc_battery_init(battery_adc_callback);
}