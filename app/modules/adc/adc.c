#include "arm_math.h"

#include "nrf.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_timer.h"
#include "sdk_config.h"

#include "acc.h"
#include "adc.h"
#include "adc_cb.h"
#include "boards.h"
#include "data_acq.h"
#include "debug.h"

#define SAADC_BUF_MAX_LEN SAADC_MEDIC_BUF_LEN

typedef struct saadc_tmr_cfg_tag
{
    uint32_t              period; // In µs
    nrf_timer_frequency_t frequency;
} saadc_tmr_cfg_t;

static nrf_ppi_channel_t     m_ppi_channel;           // PPI Instance for comparaison of SAADC Buffer
static nrf_ppi_channel_t     m_saadc_buf_swap_ppi_ch; // Additional PPI channel to trigger the START task on END event in order to prevent a
                                                      // buffer order swap in the SAADC
static const nrf_drv_timer_t m_saadc_tmr = NRF_DRV_TIMER_INSTANCE(3);
static nrf_saadc_value_t     m_saadc_buf[2][SAADC_BUF_MAX_LEN];
static volatile uint8_t      m_adc_initialized;

static void pan_212_workaround(uint32_t flag)
{
    // PAN-212 workaround
    // https://devzone.nordicsemi.com/f/nordic-q-a/39856/saadc-scan-burst-oversmaple
    // https://devzone.nordicsemi.com/f/nordic-q-a/45339/saadc-burst-problems-in-scan-vs-non-scan-acquisitions/
    *((volatile uint32_t *)0x40007FFC) = flag;
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

static void adc_medic_channels_init(void)
{
    ret_code_t err_code;
/*
    nrf_saadc_channel_config_t ecg_channel_cfg = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(ECG_IN);
    ecg_channel_cfg.gain                       = NRF_SAADC_GAIN1_4;
    ecg_channel_cfg.reference                  = NRF_SAADC_REFERENCE_INTERNAL;
    ecg_channel_cfg.acq_time                   = NRF_SAADC_ACQTIME_40US;
    ecg_channel_cfg.burst                      = NRF_SAADC_BURST_ENABLED;
    err_code                                   = nrf_drv_saadc_channel_init(SAADC_CH_0, &ecg_channel_cfg);
    APP_ERROR_CHECK(err_code);
*/
    nrf_saadc_channel_config_t breath0_channel_cfg = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(RESPI_UC_0);
    breath0_channel_cfg.gain                       = NRF_SAADC_GAIN1_4;
    breath0_channel_cfg.reference                  = NRF_SAADC_REFERENCE_INTERNAL;
    breath0_channel_cfg.acq_time                   = NRF_SAADC_ACQTIME_40US;
    breath0_channel_cfg.burst                      = NRF_SAADC_BURST_ENABLED;
    err_code                                       = nrf_drv_saadc_channel_init(SAADC_CH_0, &breath0_channel_cfg);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t breath1_channel_cfg = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(RESPI_UC_1);
    breath1_channel_cfg.gain                       = NRF_SAADC_GAIN1_4;
    breath1_channel_cfg.reference                  = NRF_SAADC_REFERENCE_INTERNAL;
    breath1_channel_cfg.acq_time                   = NRF_SAADC_ACQTIME_40US;
    breath1_channel_cfg.burst                      = NRF_SAADC_BURST_ENABLED;
    err_code                                       = nrf_drv_saadc_channel_init(SAADC_CH_1, &breath1_channel_cfg);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t temp0_channel_cfg = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(TEMP0);
    temp0_channel_cfg.gain                       = NRF_SAADC_GAIN1_4;
    temp0_channel_cfg.reference                  = NRF_SAADC_REFERENCE_VDD4;
    temp0_channel_cfg.acq_time                   = NRF_SAADC_ACQTIME_40US;
    temp0_channel_cfg.burst                      = NRF_SAADC_BURST_ENABLED;
    err_code                                     = nrf_drv_saadc_channel_init(SAADC_CH_2, &temp0_channel_cfg);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t temp1_channel_cfg = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(TEMP1);
    temp1_channel_cfg.gain                       = NRF_SAADC_GAIN1_4;
    temp1_channel_cfg.reference                  = NRF_SAADC_REFERENCE_VDD4;
    temp1_channel_cfg.acq_time                   = NRF_SAADC_ACQTIME_40US;
    temp1_channel_cfg.burst                      = NRF_SAADC_BURST_ENABLED;
    err_code                                     = nrf_drv_saadc_channel_init(SAADC_CH_3, &temp1_channel_cfg);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t battery_channel_cfg = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(VBAT);
    battery_channel_cfg.gain                       = NRF_SAADC_GAIN1_4;
    battery_channel_cfg.reference                  = NRF_SAADC_REFERENCE_VDD4;
    battery_channel_cfg.acq_time                   = NRF_SAADC_ACQTIME_40US;
    battery_channel_cfg.burst                      = NRF_SAADC_BURST_ENABLED;
    err_code                                       = nrf_drv_saadc_channel_init(SAADC_CH_4, &battery_channel_cfg);
    APP_ERROR_CHECK(err_code);
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

static void adc_battery_channels_init(void)
{
    ret_code_t err_code;

    nrf_saadc_channel_config_t battery_channel_cfg = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(VBAT);
    battery_channel_cfg.gain                       = NRF_SAADC_GAIN1_4;
    battery_channel_cfg.reference                  = NRF_SAADC_REFERENCE_VDD4;
    battery_channel_cfg.acq_time                   = NRF_SAADC_ACQTIME_40US;
    battery_channel_cfg.burst                      = NRF_SAADC_BURST_ENABLED;
    err_code                                       = nrf_drv_saadc_channel_init(SAADC_CH_0, &battery_channel_cfg);
    APP_ERROR_CHECK(err_code);
}

static void adc_imp_channels_init(void)
{
    /*
    ret_code_t err_code;

    nrf_saadc_channel_config_t imp_channel_cfg = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(MES_ZTH);
    imp_channel_cfg.gain                       = NRF_SAADC_GAIN1_5;
    imp_channel_cfg.reference                  = NRF_SAADC_REFERENCE_INTERNAL;
    imp_channel_cfg.acq_time                   = NRF_SAADC_ACQTIME_5US;
    imp_channel_cfg.burst                      = NRF_SAADC_BURST_DISABLED;
    err_code                                   = nrf_drv_saadc_channel_init(SAADC_CH_5, &imp_channel_cfg);
    APP_ERROR_CHECK(err_code);
    */
}

static void adc_temp_channels_init(void)
{
    ret_code_t err_code;

    nrf_saadc_channel_config_t temp0_channel_cfg = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(TEMP0);
    temp0_channel_cfg.gain                       = NRF_SAADC_GAIN1_4;
    temp0_channel_cfg.reference                  = NRF_SAADC_REFERENCE_VDD4;
    temp0_channel_cfg.acq_time                   = NRF_SAADC_ACQTIME_40US;
    temp0_channel_cfg.burst                      = NRF_SAADC_BURST_ENABLED;
    err_code                                     = nrf_drv_saadc_channel_init(SAADC_CH_0, &temp0_channel_cfg);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t temp1_channel_cfg = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(TEMP1);
    temp1_channel_cfg.gain                       = NRF_SAADC_GAIN1_4;
    temp1_channel_cfg.reference                  = NRF_SAADC_REFERENCE_VDD4;
    temp1_channel_cfg.acq_time                   = NRF_SAADC_ACQTIME_40US;
    temp1_channel_cfg.burst                      = NRF_SAADC_BURST_ENABLED;
    err_code                                     = nrf_drv_saadc_channel_init(SAADC_CH_1, &temp1_channel_cfg);
    APP_ERROR_CHECK(err_code);
}

void adc_uninit(void)
{
    if (m_adc_initialized)
    {
        nrf_drv_timer_disable(&m_saadc_tmr);

        ret_code_t err_code = nrf_drv_ppi_channel_disable(m_ppi_channel);
        APP_ERROR_CHECK(err_code);
        err_code = nrf_drv_ppi_channel_disable(m_saadc_buf_swap_ppi_ch);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_drv_ppi_channel_free(m_ppi_channel);
        APP_ERROR_CHECK(err_code);
        err_code = nrf_drv_ppi_channel_free(m_saadc_buf_swap_ppi_ch);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_drv_ppi_uninit();
        APP_ERROR_CHECK(err_code);

        nrf_drv_timer_uninit(&m_saadc_tmr);

        nrf_drv_saadc_abort();
        while (nrf_drv_saadc_is_busy())
            ;
        nrf_drv_saadc_uninit();
        pan_212_workaround(0);

        m_adc_initialized = 0;
    }
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

static void saadc_tmr_handler(nrf_timer_event_t event_type, void *p_context) {}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

static void adc_sampling_evt_init(const saadc_tmr_cfg_t *const saadc_tmr_cfg)
{
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t tmr_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    tmr_cfg.frequency              = saadc_tmr_cfg->frequency;

    err_code = nrf_drv_timer_init(&m_saadc_tmr, &tmr_cfg, saadc_tmr_handler); // Setup m_saadc_tmr for compare event
    APP_ERROR_CHECK(err_code);

    uint32_t ticks = nrf_drv_timer_us_to_ticks(&m_saadc_tmr, saadc_tmr_cfg->period);
    nrf_drv_timer_extended_compare(&m_saadc_tmr, NRF_TIMER_CC_CHANNEL0, ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
    nrf_drv_timer_enable(&m_saadc_tmr);

    uint32_t tmr_compare_evt_addr  = nrf_drv_timer_compare_event_address_get(&m_saadc_tmr, NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_evt_addr = nrf_drv_saadc_sample_task_get();

    // Setup ppi channel so that timer compare event is triggering sample task in SAADC
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_ppi_channel_alloc(&m_saadc_buf_swap_ppi_ch);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel, tmr_compare_evt_addr, saadc_sample_evt_addr);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_ppi_channel_assign(m_saadc_buf_swap_ppi_ch, (uint32_t)&NRF_SAADC->EVENTS_END, (uint32_t)&NRF_SAADC->TASKS_START);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_ppi_channel_enable(m_saadc_buf_swap_ppi_ch);
    APP_ERROR_CHECK(err_code);
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void adc_medic_init(nrf_drv_saadc_event_handler_t event_handler)
{
    ret_code_t             err_code;
    nrf_drv_saadc_config_t saadc_cfg = NRF_DRV_SAADC_DEFAULT_CONFIG;

    adc_uninit();

    saadc_cfg.resolution         = NRF_SAADC_RESOLUTION_12BIT;
    saadc_cfg.oversample         = NRF_SAADC_OVERSAMPLE_8X;
    saadc_cfg.interrupt_priority = APP_IRQ_PRIORITY_HIGHEST; // APP_IRQ_PRIORITY_LOW;
    pan_212_workaround(1);
    err_code = nrf_drv_saadc_init(&saadc_cfg, event_handler);
    APP_ERROR_CHECK(err_code);

    adc_medic_channels_init();

    // Set SAADC buffer 0. The SAADC will start to write to this buffer
    err_code = nrf_drv_saadc_buffer_convert(m_saadc_buf[0], SAADC_MEDIC_BUF_LEN);
    APP_ERROR_CHECK(err_code);

    // Set SAADC buffer 1. The SAADC will write to this buffer when buffer 0 is full. This will give the applicaiton time to process data in
    // buffer 0.
    err_code = nrf_drv_saadc_buffer_convert(m_saadc_buf[1], SAADC_MEDIC_BUF_LEN);
    APP_ERROR_CHECK(err_code);

    saadc_tmr_cfg_t saadc_tmr_cfg = {.period = SAADC_MEDIC_SAMPLE_RATE, .frequency = NRF_TIMER_FREQ_250kHz};
    adc_sampling_evt_init(&saadc_tmr_cfg);
    m_adc_initialized = 1;

    INFO("Medic. sampling...");
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void adc_battery_init(nrf_drv_saadc_event_handler_t event_handler)
{
    ret_code_t             err_code;
    nrf_drv_saadc_config_t saadc_cfg = NRF_DRV_SAADC_DEFAULT_CONFIG;

    adc_uninit();

    saadc_cfg.resolution         = NRF_SAADC_RESOLUTION_12BIT;
    saadc_cfg.oversample         = NRF_SAADC_OVERSAMPLE_DISABLED;
    saadc_cfg.interrupt_priority = APP_IRQ_PRIORITY_HIGHEST; // APP_IRQ_PRIORITY_LOW;
    pan_212_workaround(1);
    err_code = nrf_drv_saadc_init(&saadc_cfg, event_handler);
    APP_ERROR_CHECK(err_code);

    adc_battery_channels_init();

    // Set SAADC buffer 0. The SAADC will start to write to this buffer
    err_code = nrf_drv_saadc_buffer_convert(m_saadc_buf[0], SAADC_BATTERY_BUF_LEN);
    APP_ERROR_CHECK(err_code);

    // Set SAADC buffer 1. The SAADC will write to this buffer when buffer 0 is full. This will give the applicaiton time to process data in
    // buffer 0.
    err_code = nrf_drv_saadc_buffer_convert(m_saadc_buf[1], SAADC_BATTERY_BUF_LEN);
    APP_ERROR_CHECK(err_code);

    saadc_tmr_cfg_t saadc_tmr_cfg = {.period = SAADC_BATTERY_SAMPLE_RATE, .frequency = NRF_TIMER_FREQ_250kHz};
    adc_sampling_evt_init(&saadc_tmr_cfg);
    m_adc_initialized = 1;

    INFO("Battery sampling...");
}

void adc_imp_init(nrf_drv_saadc_event_handler_t event_handler)
{
    ret_code_t             err_code;
    nrf_drv_saadc_config_t saadc_cfg = NRF_DRV_SAADC_DEFAULT_CONFIG;

    adc_uninit();

    saadc_cfg.resolution         = NRF_SAADC_RESOLUTION_12BIT;
    saadc_cfg.oversample         = NRF_SAADC_OVERSAMPLE_DISABLED;
    saadc_cfg.interrupt_priority = APP_IRQ_PRIORITY_HIGHEST; // APP_IRQ_PRIORITY_LOW;
    pan_212_workaround(1);
    err_code = nrf_drv_saadc_init(&saadc_cfg, event_handler);
    APP_ERROR_CHECK(err_code);

    adc_imp_channels_init();

    // Set SAADC buffer 0. The SAADC will start to write to this buffer
    err_code = nrf_drv_saadc_buffer_convert(m_saadc_buf[0], SAADC_IMP_BUF_LEN);
    APP_ERROR_CHECK(err_code);

    // Set SAADC buffer 1. The SAADC will write to this buffer when buffer 0 is full. This will give the applicaiton time to process data
    // in buffer 0.
    err_code = nrf_drv_saadc_buffer_convert(m_saadc_buf[1], SAADC_IMP_BUF_LEN);
    APP_ERROR_CHECK(err_code);

    saadc_tmr_cfg_t saadc_tmr_cfg = {.period = SAADC_IMP_SAMPLE_RATE, .frequency = NRF_TIMER_FREQ_500kHz};
    adc_sampling_evt_init(&saadc_tmr_cfg);
    m_adc_initialized = 1;

    INFO("Imp. sampling...");
}

void adc_temp_init(nrf_drv_saadc_event_handler_t event_handler)
{
    ret_code_t             err_code;
    nrf_drv_saadc_config_t saadc_cfg = NRF_DRV_SAADC_DEFAULT_CONFIG;

    adc_uninit();

    saadc_cfg.resolution         = NRF_SAADC_RESOLUTION_12BIT;
    saadc_cfg.oversample         = NRF_SAADC_OVERSAMPLE_8X;
    saadc_cfg.interrupt_priority = APP_IRQ_PRIORITY_HIGHEST; // APP_IRQ_PRIORITY_LOW;
    pan_212_workaround(1);
    err_code = nrf_drv_saadc_init(&saadc_cfg, event_handler);
    APP_ERROR_CHECK(err_code);

    adc_temp_channels_init();

    // Set SAADC buffer 0. The SAADC will start to write to this buffer
    err_code = nrf_drv_saadc_buffer_convert(m_saadc_buf[0], SAADC_TEMP_BUF_LEN);
    APP_ERROR_CHECK(err_code);

    // Set SAADC buffer 1. The SAADC will write to this buffer when buffer 0 is full. This will give the applicaiton time to process data in
    // buffer 0.
    err_code = nrf_drv_saadc_buffer_convert(m_saadc_buf[1], SAADC_TEMP_BUF_LEN);
    APP_ERROR_CHECK(err_code);

    saadc_tmr_cfg_t saadc_tmr_cfg = {.period = SAADC_TEMP_SAMPLE_RATE, .frequency = NRF_TIMER_FREQ_250kHz};
    adc_sampling_evt_init(&saadc_tmr_cfg);
    m_adc_initialized = 1;

    INFO("Temp. sampling...");
}

/**
 * @brief Function copying data from a src buffer to a dest buffer
 *
 * @param[out] dest Destination buffer address
 * @param[in] src Source buffer address
 * @param[in] len Number of elements to copy
 * @param[in] offset Source buffer offset
 * @param[in] step Source buffer index increment step
 */
void adc_channel_buf_cpy(nrf_saadc_value_t *const dest, nrf_saadc_value_t const *const src, uint16_t len, uint16_t offset, uint16_t step)
{
    if (!dest || !src || !step)
    {
        INFO("[%s] > Invalid params!", (uint32_t) __func__);
        return;
    }

    memset((uint8_t *)dest, 0, len * sizeof(nrf_saadc_value_t));

    for (uint16_t i = 0; i < len; i++)
    {
        nrf_saadc_value_t val = src[offset + (i * step)];
        dest[i]               = (val < 0) ? 0 : (val & 0x0FFF);
    }
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

float64_t adc_buffer_avg(nrf_saadc_value_t const *const data, uint8_t len)
{
    uint32_t sum = 0;

    for (uint8_t i = 0; i < len; i++)
    {
        sum += data[i];
    }

    float64_t avg = (float64_t)sum / len;

    if (isfinite(avg))
    {
        return avg;
    }

    INFO("[%s] Floating-point computation error!", (uint32_t) __func__);
    return 0.0;
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void adc_medic_meas_init(void)
{
    adc_medic_init(saadc_callback);
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void adc_meas_stop(void)
{
    adc_uninit();
    NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos);
    sd_nvic_ClearPendingIRQ(SAADC_IRQn);
    acc_IT_on();
}