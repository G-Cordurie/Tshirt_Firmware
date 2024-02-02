#ifndef IMP_H
#define IMP_H

#include "adg714.h"
#include "dac_spi.h"
#include "nrf_drv_saadc.h"

#define IMP_STIM_SIN_PT_NBR    20 // Impedance stimulation sinusoid points number

/** Number of point we use and so number of waves in the sin use for
 * impedance must be > 160 as at least 8 waves * 20 points in = 160 3
 * last weave for measure waves and 1 previously empty */
#define IMP_NB_WAVES           400

#define CH0_ON_CMD             0x88
#define CH1_ON_CMD             0x84
#define CH2_ON_CMD             0x48
#define CH3_ON_CMD             0x44
#define SWITCH_OFF_CMD         0x00

#define IMP_SAMPLING_PERIOD_MS 600000U

typedef enum imp_meas_state_tag
{
    IMP_STANDBY = 0,
    IMP_START   = 1,
    IMP_RUNNING = 2,
    IMP_STOP    = 3,
    NBR_IMP_STATE
} imp_meas_state_t;

typedef struct imp_meas_ctx_tag
{
    volatile uint16_t         imp_measuring;
    volatile imp_meas_state_t imp_state;
} imp_meas_ctx_t;

typedef enum
{
    IMP_CH0 = 0,
    IMP_CH1,
    IMP_CH2,
    IMP_CH3,
    IMP_CH_NBR
} imp_ch_id_t;

extern uint16_t STIM[IMP_STIM_SIN_PT_NBR];

void             imp_stimulus_init(void);
void             imp_ch_switch_on(imp_ch_id_t chId);
void             imp_ch_switch_off(void);
void             imp_meas_init(void);
void             imp_meas_enable(void);
void             imp_data_process(nrf_drv_saadc_evt_t const *evt, uint8_t *ch);
void             imp_adc_callback(nrf_drv_saadc_evt_t const *p_event);
void             imp_get_meas(uint8_t *const data, uint8_t len);
void             imp_meas_start(void);
void             imp_meas_run(void);
void             imp_meas_stop(void);
void             imp_meas_standby(void);
imp_meas_state_t imp_meas_state_get(void);

#endif // IMPEDANCE_H
