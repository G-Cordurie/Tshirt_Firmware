#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "arm_math.h"

#include "crc32.h"

#include "acc.h"
#include "adc.h"
#include "app_ble_gap.h"
#include "battery.h"
#include "dac.h"
#include "data_acq.h"
#include "debug.h"
#include "imp.h"
#include "pwr_reg.h"
#include "rtc.h"
#include "spi_flash.h"
#include "temp.h"

#define SAADC_MAX_VAL              4095U
#define SAADC_MIN_VAL              0U

#define ECG_SIN_PERIOD_MS          1000U
#define ECG_ADC_SAMPLE_RATE_MS     50U
#define ECG_SAMPLES_PER_PERIOD     (ECG_SIN_PERIOD_MS / ECG_ADC_SAMPLE_RATE_MS)
#define ECG_MIN_ADC_VAL            0U

#define BREATH_SIG_PERIOD_MS       5000U
#define BREATH_ADC_SAMPLE_RATE_MS  50U
#define BREATH_SAMPLES_PER_PERIOD  (BREATH_SIG_PERIOD_MS / BREATH_ADC_SAMPLE_RATE_MS)
#define BREATH_MIN_ADC_VAL         0U

// ECG thresholds
#define ECG_SIN_MIN_PERIOD_MS      900U
#define ECG_SIN_MAX_PERIOD_MS      1100U
#define ECG_SIN_MIN_PP_AMP         100U
#define ECG_SIN_MAX_PP_AMP         250U
#define ECG_SIN_VALID_PERCENT      ((float64_t)60.0)

// Breath thresholds
#define BREATH_ADC_VAL_MIN         ((uint16_t)1800)
#define BREATH_ADC_VAL_MAX         ((uint16_t)2000)

// Temp thresholds
#define LDO_ON_TEMP_VAL_MIN        ((int16_t)2000)
#define LDO_ON_TEMP_VAL_MAX        ((int16_t)3000)

// LDO thresholds
#define LDO_OFF_CTN_VAL_MAX        ((float64_t)3000.0)

// Battery thresholds
#define BATTERY_VOLT_MIN           ((float64_t)3.0)
#define BATTERY_VOLT_MAX           ((float64_t)5.0)

#define IMP_VAL_MIN                100U
#define IMP_VAL_MAX                200U

#define FLASH_DEV_CODE             0x21U
#define AD_FLASH_MAGIC_VAL         0xA5A5A5A5A5A5A5A5U

#define ACC_DEV_ID                 0x33U

#define ALL_TESTS_MASK             0x7FFU

#define MEDIC_TEST_TIMEOUT_MS      (60U * 1000U)
#define IMP_TEST_TIMEOUT_MS        (5U * 1000U)
#define LDO_SWITCH_TEST_TIMEOUT_MS (10U * 1000U)

#define PASS                       1U
#define FAIL                       0U

#define AUTO_DIAG_SET_STATUS(st_field, val, min, max)                                                                                      \
    do                                                                                                                                     \
    {                                                                                                                                      \
        if (((val) >= (min)) && ((val) <= (max)))                                                                                          \
        {                                                                                                                                  \
            st_field = PASS;                                                                                                               \
        }                                                                                                                                  \
        else                                                                                                                               \
        {                                                                                                                                  \
            st_field = FAIL;                                                                                                               \
        }                                                                                                                                  \
    } while (0)

typedef enum status_tag
{
    status_passed = 0x10U,
    status_failed,
    status_skip
} status_t;

typedef struct sample_tag
{
    uint16_t val;
    uint64_t timestamp;
} sample_t;

typedef struct peak_tag
{
    sample_t min;
    sample_t max;
} peak_t;

typedef struct sig_prop_tag
{
    peak_t   peak0;
    peak_t   peak1;
    uint32_t sample_cntr;
    uint32_t samples_per_period;
} sig_prop_t;

typedef struct sig_detect_info_tag
{
    uint32_t   min_period;
    uint32_t   max_period;
    uint16_t   min_pp_amp;
    uint16_t   max_pp_amp;
    uint32_t   period_cntr;
    uint32_t   valid_period_cntr;
    sig_prop_t sig_prop;
} sig_detect_info_t;

typedef struct temp_tag
{
    float64_t ntc;
    int16_t   degCls;
} temp_t;

typedef struct auto_diag_fields_tag
{
    uint16_t ecg            : 1;
    uint16_t breath0        : 1;
    uint16_t breath1        : 1;
    uint16_t temp0          : 1;
    uint16_t temp1          : 1;
    uint16_t imp            : 1;
    uint16_t battery_volt   : 1;
    uint16_t ldo_switch     : 1;
    uint16_t flash          : 1;
    uint16_t acc            : 1;
    uint16_t battery_detect : 1;
    uint16_t result         : 5;
} auto_diag_fields_t;

typedef union auto_diag_status_tag
{
    uint16_t           value;
    auto_diag_fields_t fields;
} auto_diag_status_t;

typedef struct auto_diag_tag
{
    uint32_t           crc;
    auto_diag_status_t status;
} auto_diag_t;

#define AUTO_DIAG_STATUS_OFFSET (offsetof(auto_diag_t, status))

static sig_detect_info_t    m_ecg_sig     = {.min_period                  = ECG_SIN_MIN_PERIOD_MS,
                                             .max_period                  = ECG_SIN_MAX_PERIOD_MS,
                                             .min_pp_amp                  = ECG_SIN_MIN_PP_AMP,
                                             .max_pp_amp                  = ECG_SIN_MAX_PP_AMP,
                                             .period_cntr                 = 0,
                                             .valid_period_cntr           = 0,
                                             .sig_prop.sample_cntr        = 0,
                                             .sig_prop.samples_per_period = ECG_SAMPLES_PER_PERIOD};
static peak_t               m_breath0_sig = {.min.val = SAADC_MAX_VAL, .max.val = SAADC_MIN_VAL};
static peak_t               m_breath1_sig = {.min.val = SAADC_MAX_VAL, .max.val = SAADC_MIN_VAL};
static temp_t               m_temp0;
static temp_t               m_temp1;
static float64_t            m_battery_volt;
static volatile auto_diag_t m_auto_diag = {.status = {.fields.battery_detect = PASS, .fields.ldo_switch = PASS}};

static uint8_t auto_diag_adc_sig_period(uint16_t adc_val, sig_prop_t *const sig_prop)
{
    if (sig_prop->sample_cntr == 0)
    {
        // Peak0
        sig_prop->peak0.max.val       = SAADC_MIN_VAL;
        sig_prop->peak0.max.timestamp = 0;
        sig_prop->peak0.min.val       = SAADC_MAX_VAL;
        sig_prop->peak0.min.timestamp = 0;

        // Peak1
        sig_prop->peak1.max.val       = SAADC_MIN_VAL;
        sig_prop->peak1.max.timestamp = 0;
        sig_prop->peak1.min.val       = SAADC_MAX_VAL;
        sig_prop->peak1.min.timestamp = 0;
    }

    if (sig_prop->sample_cntr < sig_prop->samples_per_period) // Peak0
    {
        // Max
        if (adc_val > sig_prop->peak0.max.val)
        {
            sig_prop->peak0.max.val       = adc_val;
            sig_prop->peak0.max.timestamp = rtc_timestamp_get();
        }

        // Min
        if (adc_val < sig_prop->peak0.min.val)
        {
            sig_prop->peak0.min.val       = adc_val;
            sig_prop->peak0.min.timestamp = rtc_timestamp_get();
        }

        sig_prop->sample_cntr++;
    }
    else if (sig_prop->sample_cntr < (2 * sig_prop->samples_per_period)) // Peak1
    {
        // Max
        if (adc_val > sig_prop->peak1.max.val)
        {
            sig_prop->peak1.max.val       = adc_val;
            sig_prop->peak1.max.timestamp = rtc_timestamp_get();
        }

        // Min
        if (adc_val < sig_prop->peak1.min.val)
        {
            sig_prop->peak1.min.val       = adc_val;
            sig_prop->peak1.min.timestamp = rtc_timestamp_get();
        }

        sig_prop->sample_cntr++;
    }

    if (sig_prop->sample_cntr >= (2 * sig_prop->samples_per_period))
    {
        sig_prop->sample_cntr = 0;
        return 1;
    }

    return 0;
}

static uint8_t auto_diag_get_sig_period(nrf_saadc_value_t const *const data, uint8_t len, sig_detect_info_t *const sig_detect_info)
{
    uint8_t ret;

    if (!data || !len || !sig_detect_info)
    {
        INFO("[AUTO-DIAG] > Function: %s, invalid params!", (uint32_t) __func__);
        return 0;
    }

    if ((ret = auto_diag_adc_sig_period((uint16_t)(adc_buffer_avg(data, len)), (sig_prop_t *const)&sig_detect_info->sig_prop)))
    {
        sig_detect_info->period_cntr++;

        uint16_t pp0_amp         = sig_detect_info->sig_prop.peak0.max.val - sig_detect_info->sig_prop.peak0.min.val;
        uint16_t pp1_amp         = sig_detect_info->sig_prop.peak1.max.val - sig_detect_info->sig_prop.peak1.min.val;
        uint64_t peak_max_period = sig_detect_info->sig_prop.peak1.max.timestamp - sig_detect_info->sig_prop.peak0.max.timestamp;
        uint64_t peak_min_period = sig_detect_info->sig_prop.peak1.min.timestamp - sig_detect_info->sig_prop.peak0.min.timestamp;

        // INFO("[AUTO-DIAG] > PP amplitudes: %u (PP0), %u (PP1)", pp0_amp, pp1_amp);
        // INFO("[AUTO-DIAG] > Peak periods: %u ms (PMax), %u ms (PMin)", (uint32_t)peak_max_period, (uint32_t)peak_min_period);

        // Amplitude
        if (((pp0_amp >= sig_detect_info->min_pp_amp) && (pp0_amp <= sig_detect_info->max_pp_amp)) &&
            ((pp1_amp >= sig_detect_info->min_pp_amp) && (pp1_amp <= sig_detect_info->max_pp_amp)))
        {
            // Period
            if (((peak_max_period >= (uint64_t)sig_detect_info->min_period) &&
                 (peak_max_period <= (uint64_t)sig_detect_info->max_period)) &&
                ((peak_min_period >= (uint64_t)sig_detect_info->min_period) && (peak_min_period <= (uint64_t)sig_detect_info->max_period)))
            {
                sig_detect_info->valid_period_cntr++;
            }
        }
    }

    return ret;
}

static void auto_diag_get_breath(nrf_saadc_value_t const *const data, uint8_t len, peak_t *const sig_peak)
{

    if (!data || !len || !sig_peak)
    {
        INFO("[AUTO-DIAG] > Function: %s, invalid params!", (uint32_t) __func__);
        return;
    }

    uint16_t adc_val = (uint16_t)adc_buffer_avg(data, len);

    // Max
    if (adc_val > sig_peak->max.val)
    {
        sig_peak->max.val = adc_val;
    }

    // Min
    if (adc_val < sig_peak->min.val)
    {
        sig_peak->min.val = adc_val;
    }
}

static void auto_diag_get_temp(nrf_saadc_value_t const *const data, uint8_t len, temp_t *const temp)
{
    if (!data || !len || !temp)
    {
        INFO("[AUTO-DIAG] > Function: %s, invalid params!", (uint32_t) __func__);
        return;
    }

    temp->ntc    = temp_convert_adc_to_ntc(adc_buffer_avg(data, len));
    temp->degCls = temp_convert_ntc_to_temp(temp->ntc);
}

static void auto_diag_get_battery_volt(nrf_saadc_value_t const *const data, uint8_t len, float64_t *const battery_volt)
{
    float64_t avg;

    if (!data || !len || !battery_volt)
    {
        INFO("[AUTO-DIAG] > Function: %s, invalid params!", (uint32_t) __func__);
        return;
    }

    avg = adc_buffer_avg(data, len);

    *battery_volt = battery_adc_to_raw_volt_convert(avg);
}

static bool auto_diag_flash_rdWr(void)
{
    if (flash_erase_block(MIN_AUTO_DIAG_ADDR) != NRF_SUCCESS)
    {
        INFO("[AUTO-DIAG] > Function: %s, Flash block erase failed!", (uint32_t) __func__);
        return false;
    }

    uint64_t data = AD_FLASH_MAGIC_VAL;
    if (flash_page_write(MIN_AUTO_DIAG_ADDR, (uint8_t *)&data, sizeof(data)) != NRF_SUCCESS)
    {
        INFO("[AUTO-DIAG] > Function: %s, Flash write failed!", (uint32_t) __func__);
        return false;
    }

    data = 0;
    if (flash_page_read(MIN_AUTO_DIAG_ADDR, (uint8_t *)&data, sizeof(data)) != NRF_SUCCESS)
    {
        INFO("[AUTO-DIAG] > Function: %s, Flash read failed!", (uint32_t) __func__);
        return false;
    }

    if (flash_erase_block(MIN_AUTO_DIAG_ADDR) != NRF_SUCCESS)
    {
        INFO("[AUTO-DIAG] > Function: %s, Flash block erase failed!", (uint32_t) __func__);
        return false;
    }

    if (data == AD_FLASH_MAGIC_VAL)
    {
        return true;
    }

    return false;
}

static void auto_diag_flash(void)
{
    m_auto_diag.status.fields.flash = FAIL;
    if ((uint8_t)FLASH_DEV_CODE == flash_read_device_id())
    {
        bool invalid_block;
        for (uint16_t block = 0; block < FLASH_TOTAL_BLOCK; block++)
        {
            if ((invalid_block = block_is_invalid(block * BLOCK_SIZE)))
            {
                INFO("[AUTO-DIAG] > Flash block %u NOK!", block);
                break;
            }
        }

        if (!invalid_block && auto_diag_flash_rdWr())
        {
            m_auto_diag.status.fields.flash = PASS;
        }
    }

    INFO("[AUTO-DIAG] > FLASH: %s", m_auto_diag.status.fields.flash ? "OK" : "NOK");
}

static void auto_diag_acc(void)
{
    uint8_t cmd[ACC_CFG_CMD_LEN];

    cmd[0] = 0x8F;
    cmd[1] = 0;

    acc_spi_transfer(&cmd[0], &cmd[0], ACC_CFG_CMD_LEN);

    if ((uint8_t)ACC_DEV_ID == cmd[1])
    {
        m_auto_diag.status.fields.acc = PASS;
    }
    else
    {
        m_auto_diag.status.fields.acc = FAIL;
    }

    INFO("[AUTO-DIAG] > ACC: %s", m_auto_diag.status.fields.acc ? "OK" : "NOK");
}

static void auto_diag_medic_adc_callback(nrf_drv_saadc_evt_t const *p_event)
{
    ret_code_t err_code;

    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        if (p_event->data.done.size == SAADC_MEDIC_BUF_LEN)
        {
            nrf_saadc_value_t adc_buffer[SAADC_MEDIC_CH_SAMPLES];

            err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_MEDIC_BUF_LEN);
            APP_ERROR_CHECK(err_code);

            // ECG
            adc_channel_buf_cpy(adc_buffer, p_event->data.done.p_buffer, SAADC_MEDIC_CH_SAMPLES, SAADC_CH_0, SAADC_MEDIC_CH_NBR);
            if (auto_diag_get_sig_period((nrf_saadc_value_t const *const)adc_buffer, SAADC_MEDIC_CH_SAMPLES,
                                         (sig_detect_info_t *const)&m_ecg_sig))
            {
                INFO("[AUTO-DIAG] > ECG period: %u/%u", m_ecg_sig.valid_period_cntr, m_ecg_sig.period_cntr);

                uint16_t pp0_amp = m_ecg_sig.sig_prop.peak0.max.val - m_ecg_sig.sig_prop.peak0.min.val;
                UNUSED_VARIABLE(pp0_amp);
                uint16_t pp1_amp = m_ecg_sig.sig_prop.peak1.max.val - m_ecg_sig.sig_prop.peak1.min.val;
                UNUSED_VARIABLE(pp1_amp);
                uint64_t peak_max_period = m_ecg_sig.sig_prop.peak1.max.timestamp - m_ecg_sig.sig_prop.peak0.max.timestamp;
                UNUSED_VARIABLE(peak_max_period);
                uint64_t peak_min_period = m_ecg_sig.sig_prop.peak1.min.timestamp - m_ecg_sig.sig_prop.peak0.min.timestamp;
                UNUSED_VARIABLE(peak_min_period);

                INFO("[AUTO-DIAG] > ECG PP amplitudes: %u (PP0), %u (PP1)", pp0_amp, pp1_amp);
                INFO("[AUTO-DIAG] > ECG Peak periods: %u ms (PMax), %u ms (PMin)", (uint32_t)peak_max_period, (uint32_t)peak_min_period);
            }

            // Breath 0
            adc_channel_buf_cpy(adc_buffer, p_event->data.done.p_buffer, SAADC_MEDIC_CH_SAMPLES, SAADC_CH_1, SAADC_MEDIC_CH_NBR);
            auto_diag_get_breath((nrf_saadc_value_t const *const)adc_buffer, SAADC_MEDIC_CH_SAMPLES, (peak_t *const)&m_breath0_sig);

            // Breath 1
            adc_channel_buf_cpy(adc_buffer, p_event->data.done.p_buffer, SAADC_MEDIC_CH_SAMPLES, SAADC_CH_2, SAADC_MEDIC_CH_NBR);
            auto_diag_get_breath((nrf_saadc_value_t const *const)adc_buffer, SAADC_MEDIC_CH_SAMPLES, (peak_t *const)&m_breath1_sig);

            // Temp0
            adc_channel_buf_cpy(adc_buffer, p_event->data.done.p_buffer, SAADC_MEDIC_CH_SAMPLES, SAADC_CH_3, SAADC_MEDIC_CH_NBR);
            auto_diag_get_temp((nrf_saadc_value_t const *const)adc_buffer, SAADC_MEDIC_CH_SAMPLES, (temp_t *const)&m_temp0);

            // Temp1
            adc_channel_buf_cpy(adc_buffer, p_event->data.done.p_buffer, SAADC_MEDIC_CH_SAMPLES, SAADC_CH_4, SAADC_MEDIC_CH_NBR);
            auto_diag_get_temp((nrf_saadc_value_t const *const)adc_buffer, SAADC_MEDIC_CH_SAMPLES, (temp_t *const)&m_temp1);

            // Battery
            adc_channel_buf_cpy(adc_buffer, p_event->data.done.p_buffer, SAADC_MEDIC_CH_SAMPLES, SAADC_CH_5, SAADC_MEDIC_CH_NBR);
            auto_diag_get_battery_volt((nrf_saadc_value_t const *const)adc_buffer, SAADC_MEDIC_CH_SAMPLES,
                                       (float64_t *const)&m_battery_volt);
            // INFO("[AUTO-DIAG] > BATTERY VOLT: " NRF_LOG_FLOAT_MARKER " Volts", NRF_LOG_FLOAT(m_battery_volt));
        }
    }
}

// static void auto_diag_temp_adc_callback(nrf_drv_saadc_evt_t const *p_event)
// {
//     ret_code_t err_code;

//     if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
//     {
//         if (p_event->data.done.size == SAADC_TEMP_BUF_LEN)
//         {
//             nrf_saadc_value_t adc_buffer[SAADC_TEMP_CH_SAMPLES];

//             err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_TEMP_BUF_LEN);
//             APP_ERROR_CHECK(err_code);

//             // Temp0
//             adc_channel_buf_cpy(adc_buffer, p_event->data.done.p_buffer, SAADC_TEMP_CH_SAMPLES, SAADC_CH_0, SAADC_TEMP_CH_NBR);
//             auto_diag_get_temp((nrf_saadc_value_t const *const)adc_buffer, SAADC_TEMP_CH_SAMPLES, (temp_t *const)&m_temp0);

//             // Temp1
//             adc_channel_buf_cpy(adc_buffer, p_event->data.done.p_buffer, SAADC_TEMP_CH_SAMPLES, SAADC_CH_1, SAADC_TEMP_CH_NBR);
//             auto_diag_get_temp((nrf_saadc_value_t const *const)adc_buffer, SAADC_TEMP_CH_SAMPLES, (temp_t *const)&m_temp1);

//             // INFO("Temp0: %d, Temp1: %d", (m_temp0.degCls / 100), (m_temp1.degCls / 100));
//         }
//     }
// }

bool auto_diag_enter_check(void)
{
    auto_diag_t auto_diag;

    if (flash_page_read(MIN_AUTO_DIAG_ADDR, (uint8_t *)&auto_diag, sizeof(auto_diag)) == NRF_SUCCESS)
    {
        uint32_t crc = crc32_compute((uint8_t *)&auto_diag + AUTO_DIAG_STATUS_OFFSET, sizeof(auto_diag_t) - AUTO_DIAG_STATUS_OFFSET, NULL);
        if (crc != auto_diag.crc)
        {
            INFO("[AUTO-DIAG] > Function: %s, CRC error!", (uint32_t) __func__);
        }
    }
    else
    {
        auto_diag.status.fields.result = status_failed;
        INFO("[AUTO-DIAG] > Function: %s, Flash read failed!", (uint32_t) __func__);
    }

    switch (auto_diag.status.fields.result)
    {
    case status_passed:
        INFO("[AUTO-DIAG] > PASSED");
        break;

    case status_skip:
        INFO("[AUTO-DIAG] > SKIP");
        return false;

    case status_failed:
        INFO("[AUTO-DIAG] > FAILED");
        break;

    default:
        break;
    }

    INFO("[AUTO-DIAG] > Enter auto-diag mode...");
    return true;
}

static void auto_diag_medic(uint32_t timeout)
{
    uint32_t elapsed_time_ms = 0;

    adc_medic_init(auto_diag_medic_adc_callback);

    while (elapsed_time_ms < timeout)
    {
        nrf_delay_ms(1000);
        elapsed_time_ms += 1000;
        INFO("[AUTO-DIAG] > %u s", elapsed_time_ms / 1000);
    }

    adc_uninit();

    m_auto_diag.status.fields.ecg   = FAIL;
    float64_t ecg_valid_period_perc = ((float64_t)m_ecg_sig.valid_period_cntr / m_ecg_sig.period_cntr) * 100.0;
    if (isfinite(ecg_valid_period_perc) && isgreaterequal(ecg_valid_period_perc, ECG_SIN_VALID_PERCENT))
    {
        m_auto_diag.status.fields.ecg = PASS;
    }

    uint8_t b_min_st;
    uint8_t b_max_st;

    AUTO_DIAG_SET_STATUS(b_min_st, m_breath0_sig.min.val, BREATH_ADC_VAL_MIN, BREATH_ADC_VAL_MAX);
    AUTO_DIAG_SET_STATUS(b_max_st, m_breath0_sig.max.val, BREATH_ADC_VAL_MIN, BREATH_ADC_VAL_MAX);
    m_auto_diag.status.fields.breath0 = (b_min_st && b_max_st) ? PASS : FAIL;

    AUTO_DIAG_SET_STATUS(b_min_st, m_breath1_sig.min.val, BREATH_ADC_VAL_MIN, BREATH_ADC_VAL_MAX);
    AUTO_DIAG_SET_STATUS(b_max_st, m_breath1_sig.max.val, BREATH_ADC_VAL_MIN, BREATH_ADC_VAL_MAX);
    m_auto_diag.status.fields.breath1 = (b_min_st && b_max_st) ? PASS : FAIL;

    AUTO_DIAG_SET_STATUS(m_auto_diag.status.fields.temp0, m_temp0.degCls, LDO_ON_TEMP_VAL_MIN, LDO_ON_TEMP_VAL_MAX);
    AUTO_DIAG_SET_STATUS(m_auto_diag.status.fields.temp1, m_temp1.degCls, LDO_ON_TEMP_VAL_MIN, LDO_ON_TEMP_VAL_MAX);

    m_auto_diag.status.fields.battery_volt = FAIL;
    if (isgreaterequal(m_battery_volt, BATTERY_VOLT_MIN) && islessequal(m_battery_volt, BATTERY_VOLT_MAX))
    {
        m_auto_diag.status.fields.battery_volt = PASS;
    }

    INFO("[AUTO-DIAG] > ECG: %s ", m_auto_diag.status.fields.ecg ? "OK" : "NOK");
    INFO("(" NRF_LOG_FLOAT_MARKER " %%)", NRF_LOG_FLOAT(ecg_valid_period_perc));

    INFO("[AUTO-DIAG] > BREATH0: %s (min: %u, max: %u)", m_auto_diag.status.fields.breath0 ? "OK" : "NOK", m_breath0_sig.min.val,
         m_breath0_sig.max.val);

    INFO("[AUTO-DIAG] > BREATH1: %s (min: %u, max: %u)", m_auto_diag.status.fields.breath1 ? "OK" : "NOK", m_breath1_sig.min.val,
         m_breath1_sig.max.val);

    INFO("[AUTO-DIAG] > TEMP0 (LDO ON): %s (Temp0: %d Cdeg, ", m_auto_diag.status.fields.temp0 ? "OK" : "NOK", m_temp0.degCls);
    INFO("CTN0: " NRF_LOG_FLOAT_MARKER " Ohms)", NRF_LOG_FLOAT(m_temp0.ntc));

    INFO("[AUTO-DIAG] > TEMP1 (LDO ON): %s (Temp1: %d Cdeg, ", m_auto_diag.status.fields.temp1 ? "OK" : "NOK", m_temp1.degCls);
    INFO("CTN1: " NRF_LOG_FLOAT_MARKER " Ohms)", NRF_LOG_FLOAT(m_temp1.ntc));

    INFO("[AUTO-DIAG] > BATTERY VOLT: %s ", m_auto_diag.status.fields.battery_volt ? "OK" : "NOK");
    INFO("(" NRF_LOG_FLOAT_MARKER " Volts)", NRF_LOG_FLOAT(m_battery_volt));
}

static void auto_diag_imp(uint32_t timeout)
{
    uint16_t imp[4] = {0};

    imp_meas_run();

    while ((imp_meas_state_get() != IMP_STOP) && (timeout != 0))
    {
        nrf_delay_ms(1);
        timeout--;
    }

    if (timeout == 0)
    {
        INFO("[AUTO-DIAG] > Function: %s, Impedance test timeout!", (uint32_t) __func__);
    }

    adc_uninit();
    dac_disconnect();
    imp_ch_switch_off();
    imp_meas_standby();

    imp_get_meas((uint8_t *const)imp, sizeof(imp));
    m_auto_diag.status.fields.imp = PASS;
    for (uint8_t i = 0; i < 4; i++)
    {
        if ((imp[i] < IMP_VAL_MIN) || (imp[i] > IMP_VAL_MAX))
        {
            m_auto_diag.status.fields.imp = FAIL;
            break;
        }
    }

    INFO("[AUTO-DIAG] > IMP: %s (CH0: %d,  CH1: %d,  CH2: %d, CH3: %d)", m_auto_diag.status.fields.imp ? "OK" : "NOK", imp[0], imp[1],
         imp[2], imp[3]);
}

// static void auto_diag_ldo_switch(uint32_t timeout)
// {
//     pwr_ldo_reg_disable();
//     adc_temp_init(auto_diag_temp_adc_callback);
//     nrf_delay_ms(timeout);
//     adc_uninit();

//     m_auto_diag.status.fields.ldo_switch = FAIL;
//     if (islessequal(m_temp0.ntc, LDO_OFF_CTN_VAL_MAX) && islessequal(m_temp1.ntc, LDO_OFF_CTN_VAL_MAX))
//     {
//         m_auto_diag.status.fields.ldo_switch = PASS;
//     }

//     INFO("[AUTO-DIAG] > LDO SWITCH: %s ", m_auto_diag.status.fields.ldo_switch ? "OK" : "NOK");
//     INFO("(CTN0: " NRF_LOG_FLOAT_MARKER " Ohms, CTN1: " NRF_LOG_FLOAT_MARKER " Ohms)", NRF_LOG_FLOAT(m_temp0.ntc),
//     NRF_LOG_FLOAT(m_temp1.ntc));
// }

static void auto_diag_status_store(void)
{
    if (flash_erase_block(MIN_AUTO_DIAG_ADDR) != NRF_SUCCESS)
    {
        INFO("[AUTO-DIAG] > Function: %s, Flash block erase failed!", (uint32_t) __func__);
        return;
    }

    m_auto_diag.crc = crc32_compute((uint8_t *)&m_auto_diag + AUTO_DIAG_STATUS_OFFSET, sizeof(auto_diag_t) - AUTO_DIAG_STATUS_OFFSET, NULL);

    if (flash_page_write(MIN_AUTO_DIAG_ADDR, (uint8_t *)&m_auto_diag, sizeof(m_auto_diag)) != NRF_SUCCESS)
    {
        INFO("[AUTO-DIAG] > Function: %s, Flash write failed!", (uint32_t) __func__);
        return;
    }

    auto_diag_t auto_diag = {.crc = 0, .status.value = 0};
    if (flash_page_read(MIN_AUTO_DIAG_ADDR, (uint8_t *)&auto_diag, sizeof(auto_diag)) != NRF_SUCCESS)
    {
        INFO("[AUTO-DIAG] > Function: %s, Flash read failed!", (uint32_t) __func__);
        return;
    }

    uint32_t crc = crc32_compute((uint8_t *)&auto_diag + AUTO_DIAG_STATUS_OFFSET, sizeof(auto_diag_t) - AUTO_DIAG_STATUS_OFFSET, NULL);

    if (crc != auto_diag.crc)
    {
        INFO("[AUTO-DIAG] > Function: %s, CRC error!", (uint32_t) __func__);
        return;
    }

    INFO("[AUTO-DIAG] > Test result sotred successfully.");
}

void auto_diag_system_reset(void)
{
    INFO("[AUTO-DIAG] Reset.");
    nrf_delay_ms(100);
    NVIC_SystemReset();
}

void auto_diag_set_status(uint8_t status)
{
    m_auto_diag.status.fields.result = status;
    auto_diag_status_store();
    INFO("[AUTO-DIAG] > Status set to 0x%X (%s)", m_auto_diag.status.fields.result,
         (m_auto_diag.status.fields.result == status_passed)
             ? "PASS"
             : ((m_auto_diag.status.fields.result == status_failed)
                    ? "FAIL"
                    : ((m_auto_diag.status.fields.result == status_skip) ? "SKIP" : "UNKNOWN")));
}

void auto_diag_run(void)
{
    rtc_start();
    pwr_ldo_reg_enable(); // LDO regulators ON
    auto_diag_medic(MEDIC_TEST_TIMEOUT_MS);
    rtc_stop();
    auto_diag_imp(IMP_TEST_TIMEOUT_MS);
    auto_diag_flash();
    auto_diag_acc();

    if (m_auto_diag.status.fields.temp0 == PASS && m_auto_diag.status.fields.temp1 == PASS)
    {
        temp_calib_run();
    }

    auto_diag_set_status((uint8_t)((m_auto_diag.status.value == ALL_TESTS_MASK) ? status_passed : status_failed));

    if (((uint8_t)m_auto_diag.status.fields.result) != ((uint8_t)status_passed))
    {
        device_name_base10_set((uint32_t)(m_auto_diag.status.value & ALL_TESTS_MASK));
    }
}