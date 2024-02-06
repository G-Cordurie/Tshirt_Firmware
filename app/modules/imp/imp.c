#include "nrf_gpio.h"

#include "acc.h"
#include "adc.h"
#include "boards.h"
#include "dac.h"
#include "data_acq.h"
#include "data_session.h"
#include "debug.h"
#include "imp.h"
#include "storage.h"

static uint16_t imp_v0_max[4]    = {0, 0, 0, 0};
static uint16_t imp_v0_min[4]    = {0xffff, 0xffff, 0xffff, 0xffff};
static uint16_t imp_v_max[3 * 4] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint16_t imp_v_min[3 * 4] = {0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff};
static uint16_t res_imp[4];

static imp_meas_ctx_t m_imp_meas_ctx = {.imp_measuring = 0, .imp_state = IMP_STANDBY};

// Stimulation Sinusoide for Impedance Measure
uint16_t STIM[IMP_STIM_SIN_PT_NBR] = {
    0x1ff8, 0x29d8, 0x32c0, 0x39d4, 0x3e5c, 0x3fec, 0x3e5c, 0x39d4, 0x32c0, 0x29d8,
    0x1ff8, 0x1614, 0x0d2c, 0x0618, 0x0190, 0x0000, 0x0190, 0x0618, 0x0d2c, 0x1614,
};

static const uint8_t chIdTochCmdTable[IMP_CH_NBR * ADG714_CMD_LEN] = {
    [IMP_CH0] = CH0_ON_CMD, [IMP_CH1] = CH1_ON_CMD, [IMP_CH2] = CH2_ON_CMD, [IMP_CH3] = CH3_ON_CMD};

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void imp_stimulus_init(void)
{
    static uint8_t initialized = 0;

    if (!initialized)
    {
        for (uint8_t i = 0; i < IMP_STIM_SIN_PT_NBR; i++)
        {
            STIM[i] = (STIM[i] / 2) + 1000;
        }

        initialized = 1;
    }
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void imp_ch_switch_on(imp_ch_id_t chId)
{
    if (chId >= IMP_CH_NBR)
    {
        INFO("[%s] Invalid channel ID!", __func__);
        return;
    }

    adg714_spi_write(&chIdTochCmdTable[chId], ADG714_CMD_LEN);
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void imp_ch_switch_off(void)
{
    uint8_t switch_off_cmd[ADG714_CMD_LEN] = {SWITCH_OFF_CMD};
    adg714_spi_write((uint8_t *)&switch_off_cmd, ADG714_CMD_LEN);
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void imp_meas_enable(void)
{
//    nrf_gpio_cfg_output(SELECT);
//    nrf_gpio_pin_clear(SELECT);
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void imp_meas_init(void)
{
    uint8_t dac_command[2] = {0};

    imp_meas_enable();
    dac_init();
    dac_command[0] = (STIM[0] & 0x3f00) >> 8;
    dac_command[1] = STIM[0] & 0x00ff;
    UNUSED_VARIABLE(dac_spi_transfer(&dac_command[0], 2));

    adg714_spi_init();

    // Empty globals values
    for (int i = 0; i < 4; i++)
    {
        imp_v0_max[i] = 0;
        imp_v0_min[i] = 0xffff;
    }
    for (int i = 0; i < (4 * 3); i++)
    {
        imp_v_max[i] = 0;
        imp_v_min[i] = 0xffff;
    }

    memset(res_imp, 0, sizeof(res_imp));
}

void imp_data_process(nrf_drv_saadc_evt_t const *evt, uint8_t *ch)
{
    static uint16_t imp_avg_v_max[4] = {0, 0, 0, 0};
    static uint16_t imp_avg_v_min[4] = {0, 0, 0, 0};

    uint8_t dac_command[2]                  = {0x00, 0x00};
    uint8_t imp[BLE_DIAGW_MAX_IMP_CHAR_LEN] = {0};

    dac_command[0] = 0x00 | ((STIM[(m_imp_meas_ctx.imp_measuring - 1) % 20] & 0x3f00) >> 8);
    dac_command[1] = STIM[(m_imp_meas_ctx.imp_measuring - 1) % 20] & 0x00ff;
    UNUSED_VARIABLE(dac_spi_transfer(&dac_command[0], 2));

    uint16_t adc_value = (evt->data.done.p_buffer[0] < 0) ? 0 : (evt->data.done.p_buffer[0] & 0x0FFF);

    // 80 to 100 4th empty wave
    if ((m_imp_meas_ctx.imp_measuring >= (80 + 1)) && (m_imp_meas_ctx.imp_measuring < (100 + 1)))
    {
        if (imp_v0_max[*ch] < adc_value)
        {
            imp_v0_max[*ch] = adc_value;
        }
        if (imp_v0_min[*ch] > adc_value)
        {
            imp_v0_min[*ch] = adc_value;
        }
    }
    else if ((m_imp_meas_ctx.imp_measuring >= (IMP_NB_WAVES - 60 + 1)) &&
             (m_imp_meas_ctx.imp_measuring < (IMP_NB_WAVES - 40 + 1))) // +1 as m_imp_meas_ctx.imp_measuring start at 1
    {
        if (imp_v_max[0 + (*ch * 3)] < adc_value)
        {
            imp_v_max[0 + (*ch * 3)] = adc_value;
        }
        if (imp_v_min[0 + (*ch * 3)] > adc_value)
        {
            imp_v_min[0 + (*ch * 3)] = adc_value;
        }
    }
    else if ((m_imp_meas_ctx.imp_measuring >= (IMP_NB_WAVES - 40 + 1)) &&
             (m_imp_meas_ctx.imp_measuring < (IMP_NB_WAVES - 20 + 1))) // +1 as m_imp_meas_ctx.imp_measuring start at 1
    {
        if (imp_v_max[1 + (*ch * 3)] < adc_value)
        {
            imp_v_max[1 + (*ch * 3)] = adc_value;
        }
        if (imp_v_min[1 + (*ch * 3)] > adc_value)
        {
            imp_v_min[1 + (*ch * 3)] = adc_value;
        }
    }
    else if (m_imp_meas_ctx.imp_measuring >= (IMP_NB_WAVES - 20 + 1)) // +1 as m_imp_meas_ctx.imp_measuring start at 1
    {
        if (imp_v_max[2 + (*ch * 3)] < adc_value)
        {
            imp_v_max[2 + (*ch * 3)] = adc_value;
        }
        if (imp_v_min[2 + (*ch * 3)] > adc_value)
        {
            imp_v_min[2 + (*ch * 3)] = adc_value;
        }
    }
    if ((m_imp_meas_ctx.imp_measuring >= IMP_NB_WAVES) && (*ch >= 3)) // 80 values index 1, 4 times
    {
        *ch = 0;

        // Calc average on 3 point for each measure
        for (int idx = 0; idx < 4; idx++)
        {
            imp_avg_v_max[idx] = ((imp_v_max[idx * 3] + imp_v_max[(idx * 3) + 1] + imp_v_max[(idx * 3) + 2]) / 3);
            imp_avg_v_min[idx] = ((imp_v_min[idx * 3] + imp_v_min[(idx * 3) + 1] + imp_v_min[(idx * 3) + 2]) / 3);

            int div = (imp_v0_max[idx] - imp_v0_min[idx]) - (imp_avg_v_max[idx] - imp_avg_v_min[idx]);

            if (div == 0)
            {
                res_imp[idx] = 0xffff;
            }
            else
            {
                res_imp[idx] = 470 * (imp_avg_v_max[idx] - imp_avg_v_min[idx]) / div;
            }
        }

        uint64_t timestamp = data_session_sample_timestamp_get(imp_idx_type);
        memcpy((uint8_t *)&imp[0], (uint8_t *)&timestamp, sizeof(timestamp));

        for (int idx = 0; idx < 4; idx++)
        {
            imp[(idx * 2) + TIMESTAMP_LEN]       = (res_imp[idx] & 0xff00) >> 8;
            imp[(idx * 2) + (TIMESTAMP_LEN + 1)] = res_imp[idx] & 0x00ff;
        }

        ring_store(IMP_TYPE, imp, sizeof(imp));
        imp_meas_stop();
    }
    else if ((m_imp_meas_ctx.imp_measuring >= IMP_NB_WAVES) && (*ch < 3))
    {
        m_imp_meas_ctx.imp_measuring = 1;
        (*ch)++;
    }
    else
    {
        m_imp_meas_ctx.imp_measuring++;
    }
}

void imp_get_meas(uint8_t *const data, uint8_t len)
{
    if (data && (len <= sizeof(res_imp)))
    {
        memcpy(data, res_imp, len);
    }
}

void imp_adc_callback(nrf_drv_saadc_evt_t const *p_event)
{
    static uint8_t imp_chan = IMP_CH0;

    if ((p_event->type == NRF_DRV_SAADC_EVT_DONE) && (p_event->data.done.size == SAADC_IMP_BUF_LEN))
    {
        if (m_imp_meas_ctx.imp_measuring == 0 || m_imp_meas_ctx.imp_state != IMP_RUNNING)
        {
            return;
        }

        if (m_imp_meas_ctx.imp_measuring == 1)
        {
            imp_ch_switch_off();
        }
        else if (m_imp_meas_ctx.imp_measuring == 100)
        {
            imp_ch_switch_on(imp_chan);
        }

        ret_code_t err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_IMP_BUF_LEN);
        APP_ERROR_CHECK(err_code);

        imp_data_process(p_event, &imp_chan);
    }
}

void imp_meas_start(void)
{
    m_imp_meas_ctx.imp_state = IMP_START;
}

void imp_meas_stop(void)
{
    CRITICAL_REGION_ENTER();
    {
        m_imp_meas_ctx.imp_measuring = 0;
        m_imp_meas_ctx.imp_state     = IMP_STOP;
    }
    CRITICAL_REGION_EXIT();
}

void imp_meas_standby(void)
{
    CRITICAL_REGION_ENTER();
    {
        m_imp_meas_ctx.imp_measuring = 0;
        m_imp_meas_ctx.imp_state     = IMP_STANDBY;
    }
    CRITICAL_REGION_EXIT();
}

void imp_meas_run(void)
{
    imp_meas_init();
    adc_imp_init(imp_adc_callback);

    CRITICAL_REGION_ENTER();
    {
        m_imp_meas_ctx.imp_measuring = 1;
        m_imp_meas_ctx.imp_state     = IMP_RUNNING;
    }
    CRITICAL_REGION_EXIT();
}

imp_meas_state_t imp_meas_state_get(void)
{
    return m_imp_meas_ctx.imp_state;
}