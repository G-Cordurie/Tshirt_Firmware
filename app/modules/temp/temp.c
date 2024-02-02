#include "arm_math.h"

#include <stdbool.h>

#include "nrf.h"
#include "nrf_delay.h"

#include "adc.h"
#include "debug.h"
#include "pwr_reg.h"
#include "spi_flash.h"
#include "timer.h"
#include "utils.h"

#define TEMP_MEAS_TIMEOUT_MS        (20U * 1000U)

#define RNTC_MAX                    ((float64_t)4221283.0) // NTC value at -40 °C: 4221.283 KOhms
#define RNTC_25                     ((float64_t)100000.0)  // NTC value at 25 °C: 100 KOhms
#define RNTC_MIN                    ((float64_t)2522.0)    // NTC value at 125 °C: 2.522 KOhms
#define T_MAX                       ((float64_t)125.0)
#define T_MIN                       ((float64_t)-40.0)

#define BETA                        ((float64_t)4250.0)   // NTC Beta constant
#define BETA_OFFSET                 ((float64_t)0.06)     // BETA offset added in order to have more accuracy in the range [34°C..43°C]
#define RTHR                        ((float64_t)65.0)     // Conductive thread resistance value: 65 Ohm
#define RPULLUP                     ((float64_t)100000.0) // Volt divider pull-up value: 100 KOhm
#define VCC                         ((float64_t)2.5)      // Volt divider pullup voltage: 2.5 Volt
#define VDD                         ((float64_t)3.0)      // µC VDD voltage: 3.0 Volt
#define R0                          RNTC_25
#define T0                          ((float64_t)25.0)
#define KLVN_CONST                  ((float64_t)273.15)
#define T0_KLVN                     ((float64_t)(KLVN_CONST + T0))

#define KLVN_CONST_PLUS_BETA_OFFSET ((float64_t)273.21)      // KLVN_CONST + BETA_OFFSET
#define INV_T0_KLVN                 ((float64_t)0.003354016) // 1 / T0_KLVN

#define ADC_GAIN1_4                 ((float64_t)0.25) // nRF SAADC gain factor 1/4
#define ADC_MAX_RES_VAL             ((float64_t)4096.0)

#define CALIB_COEF_MIN_VAL          ((float64_t)0.81201171875)
#define CALIB_COEF_NOM_VAL          ((float64_t)0.833333333) // (VCC * ADC_GAIN1_4) / (VDD / 4)
#define CALIB_COEF_MAX_VAL          ((float64_t)0.85546875)

#define TEMP_ADC_BUFFER_LEN         20U

#define TEMP_CALIB_DONE             0xAA55AA55U

typedef struct temp_calib_tag
{
    float64_t coef;
    uint32_t  status;
} temp_calib_t;

typedef struct temp_tag
{
    float64_t adc_val;
    uint32_t  idx;
    float64_t adc_buffer[TEMP_ADC_BUFFER_LEN];
} temp_t;

static volatile temp_t m_temp0;
static volatile temp_t m_temp1;

static temp_calib_t m_temp_calib = {.coef = CALIB_COEF_NOM_VAL};

static float64_t temp_adc_buffer_avg(float64_t const *const data, uint32_t len)
{
    float64_t avg = sum_double((double *)data, len) / len;

    if (isfinite(avg))
    {
        return avg;
    }

    INFO("[%s] Floating-point computation error!", (uint32_t) __func__);
    return 0.0;
}

static void temp_get_adc_val(nrf_saadc_value_t const *const data, uint8_t len, temp_t *const temp)
{
    if (!data || !len || !temp)
    {
        INFO("[%s] Invalid params!\n", (uint32_t) __func__);
        return;
    }

    temp->adc_buffer[temp->idx++] = adc_buffer_avg(data, len);

    if (temp->idx >= TEMP_ADC_BUFFER_LEN)
    {
        temp->idx = 0;
    }
}

static void temp_adc_callback(nrf_drv_saadc_evt_t const *p_event)
{
    ret_code_t err_code;

    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        if (p_event->data.done.size == SAADC_TEMP_BUF_LEN)
        {
            nrf_saadc_value_t adc_buffer[SAADC_TEMP_CH_SAMPLES] = {0};

            err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_TEMP_BUF_LEN);
            APP_ERROR_CHECK(err_code);

            // NTC0
            adc_channel_buf_cpy(adc_buffer, p_event->data.done.p_buffer, SAADC_TEMP_CH_SAMPLES, SAADC_CH_0, SAADC_TEMP_CH_NBR);
            temp_get_adc_val((nrf_saadc_value_t const *const)adc_buffer, SAADC_TEMP_CH_SAMPLES, (temp_t *const)&m_temp0);

            // NTC1
            adc_channel_buf_cpy(adc_buffer, p_event->data.done.p_buffer, SAADC_TEMP_CH_SAMPLES, SAADC_CH_1, SAADC_TEMP_CH_NBR);
            temp_get_adc_val((nrf_saadc_value_t const *const)adc_buffer, SAADC_TEMP_CH_SAMPLES, (temp_t *const)&m_temp1);
        }
    }
}

static void temp_adc_meas(uint32_t timeout)
{
    uint32_t elapsed_time_ms = 0;

    memset((temp_t *)&m_temp0, 0, sizeof(m_temp0));
    memset((temp_t *)&m_temp1, 0, sizeof(m_temp1));

    pwr_ldo_reg_enable(); // LDO regulators ON
    adc_temp_init(temp_adc_callback);

    while (elapsed_time_ms < timeout)
    {
        nrf_delay_ms(1000);
        elapsed_time_ms += 1000;
        INFO("[TEMP_CALIB] > %u s\n", elapsed_time_ms / 1000);
    }

    adc_uninit();

    m_temp0.adc_val = temp_adc_buffer_avg((float64_t const *const)&m_temp0.adc_buffer, TEMP_ADC_BUFFER_LEN);
    m_temp1.adc_val = temp_adc_buffer_avg((float64_t const *const)&m_temp1.adc_buffer, TEMP_ADC_BUFFER_LEN);

    INFO("[TEMP_CALIB] > ctn0_adc_val: " NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(m_temp0.adc_val));
    INFO("[TEMP_CALIB] > ctn1_adc_val: " NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(m_temp1.adc_val));
}

static float64_t temp_calib_coef(void)
{
    float64_t avg = avg_double(m_temp0.adc_val, m_temp1.adc_val) / 2048.0;

    if (isgreater(avg, CALIB_COEF_MAX_VAL))
    {
        return CALIB_COEF_MAX_VAL;
    }

    if (isless(avg, CALIB_COEF_MIN_VAL))
    {
        return CALIB_COEF_MIN_VAL;
    }

    return avg;
}

static void temp_calib_store(void)
{
    temp_calib_t temp_calib;

    if (flash_erase_block(MIN_TEMP_ADDR) != NRF_SUCCESS)
    {
        INFO("[TEMP_CALIB] > Function: %s, Flash block erase failed!", (uint32_t) __func__);
        return;
    }

    if (flash_page_write(MIN_TEMP_ADDR, (uint8_t *)&m_temp_calib, sizeof(m_temp_calib)) != NRF_SUCCESS)
    {
        INFO("[TEMP_CALIB] > Function: %s, Flash write failed!", (uint32_t) __func__);
        return;
    }

    if (flash_page_read(MIN_TEMP_ADDR, (uint8_t *)&temp_calib, sizeof(temp_calib)) != NRF_SUCCESS)
    {
        INFO("[TEMP_CALIB] > Function: %s, Flash read failed!", (uint32_t) __func__);
        return;
    }

    if (memcmp((uint8_t *)&temp_calib, (uint8_t *)&m_temp_calib, sizeof(temp_calib_t)))
    {
        INFO("[TEMP_CALIB] > Function: %s, Sotred data doesn't match!", (uint32_t) __func__);
    }

    INFO("[TEMP_CALIB] > Temp. calib coef (" NRF_LOG_FLOAT_MARKER ") stored successfully!", NRF_LOG_FLOAT(m_temp_calib.coef));
}

bool temp_calib_check(void)
{
    temp_calib_t temp_calib;

    if (flash_page_read(MIN_TEMP_ADDR, (uint8_t *)&temp_calib, sizeof(temp_calib)) != NRF_SUCCESS)
    {
        INFO("[TEMP_CALIB] > Function: %s, Flash read failed!", (uint32_t) __func__);
        return false;
    }

    if (temp_calib.status == TEMP_CALIB_DONE)
    {
        m_temp_calib.status = temp_calib.status;
        m_temp_calib.coef   = (isless(temp_calib.coef, CALIB_COEF_MIN_VAL) || isgreater(temp_calib.coef, CALIB_COEF_MAX_VAL))
                                  ? CALIB_COEF_NOM_VAL
                                  : temp_calib.coef;

        INFO("[TEMP_CALIB] > Temp. calib coef loaded ");
        INFO("(Coef: " NRF_LOG_FLOAT_MARKER ")", NRF_LOG_FLOAT(m_temp_calib.coef));

        return false;
    }

    INFO("[TEMP_CALIB] > Enter temp. calib. mode...");
    return true;
}

void temp_system_reset(void)
{
    INFO("[TEMP_CALIB] Reset.");
    nrf_delay_ms(100);
    NVIC_SystemReset();
}

void temp_calib_set_status(uint32_t status)
{
    m_temp_calib.status = status;
    temp_calib_store();
    INFO("[TEMP_CALIB] > Status set to 0x%X", m_temp_calib.status);
}

void temp_calib_run(void)
{
    INFO("[TEMP_CALIB] > Temperature calib. started...");
    temp_adc_meas(TEMP_MEAS_TIMEOUT_MS);
    m_temp_calib.coef = temp_calib_coef();
    temp_calib_set_status(TEMP_CALIB_DONE);
    INFO("[TEMP_CALIB] > Temperature calib. done.");
}

float64_t temp_convert_adc_to_ntc(float64_t adc_val)
{
    float64_t ntc;

    ntc = (m_temp_calib.coef * 4096.0) - adc_val;
    if (isfinite(ntc) && isgreater(ntc, 0.0))
    {
        ntc = (adc_val * RPULLUP) / ntc;
        if (isfinite(ntc))
        {
            if (isless(ntc, RNTC_MIN))
            {
                return RNTC_MIN;
            }

            if (isgreater(ntc, RNTC_MAX))
            {
                return RNTC_MAX;
            }

            return ntc;
        }
    }

    return RNTC_MAX;
}

int16_t temp_convert_ntc_to_temp(float64_t ntc_val)
{
    float64_t temp;

    temp = (ntc_val - RTHR) / R0;

    if (isfinite(temp) && isgreater(temp, 0.0))
    {
        temp = log(temp);
        if (isfinite(temp))
        {
            temp = (1 / ((temp / BETA) + INV_T0_KLVN)) - KLVN_CONST_PLUS_BETA_OFFSET;
            if (isfinite(temp))
            {
                if (isless(temp, T_MIN))
                {
                    return (int16_t)(T_MIN * 100);
                }

                if (isgreater(temp, T_MAX))
                {
                    return (int16_t)(T_MAX * 100.0);
                }

                return (int16_t)(temp * 100.0);
            }
        }
    }

    return (int16_t)(T_MAX * 100.0);
}

float64_t temp_vcc_get(void)
{
    return VCC;
}

float64_t temp_calib_coef_get(void)
{
    return m_temp_calib.coef;
}