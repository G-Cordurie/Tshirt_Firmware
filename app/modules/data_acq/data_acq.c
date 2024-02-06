#include "data_acq.h"
#include "acc.h"
#include "adc.h"
#include "app_ble_mngr.h"
#include "battery.h"
#include "battery_chrgr.h"
#include "dac.h"
#include "data_session.h"
#include "debug.h"
#include "ecg.h"
#include "imp.h"
#include "pwr_reg.h"
#include "storage.h"

typedef enum data_acq_state_tag
{
    IDLE_STATE,
    MEDIC_MEAS_STATE,
    IMP_MEAS_STATE,
    BATTERY_MEAS_STATE,
} data_acq_state_t;

static volatile data_acq_state_t m_next_state = IDLE_STATE;

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void medic_meas_start(void)
{
    adc_meas_stop();
    pwr_ldo_reg_enable();
    //battery_switch_enable();
    ecg_init();
    //dac_disconnect();
    //imp_ch_switch_off();
    acc_spi_init();
    adc_medic_meas_init();
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void medic_meas_stop(void)
{
    adc_meas_stop();
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

static data_acq_state_t idle_state_hanlder(void)
{
    if (is_ble_connected() && data_session_user_is_connected())
    {
        medic_meas_start();
        INFO("[DATA_ACQ] > Go to MEDIC_MEAS_STATE");
        return MEDIC_MEAS_STATE;
    }

    return IDLE_STATE;
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

static data_acq_state_t medic_meas_state_hanlder(void)
{
#if 0    
    if (battery_is_charging())
    {
        medic_meas_stop();
        battery_switch_enable();
        pwr_ldo_reg_disable();
        battery_meas_start();
        INFO("[DATA_ACQ] > Go to BATTERY_MEAS_STATE");
        return BATTERY_MEAS_STATE;
    }
#endif

    if (imp_meas_state_get() == IMP_START)
    {
        medic_meas_stop();
        imp_meas_run();
        INFO("[DATA_ACQ] > Go to IMP_MEAS_STATE");
        return IMP_MEAS_STATE;
    }

    return MEDIC_MEAS_STATE;
}


static data_acq_state_t imp_meas_state_hanlder(void)
{
    if (imp_meas_state_get() == IMP_STOP)
    {
        medic_meas_start();
        imp_meas_standby();
        INFO("[DATA_ACQ] > Go to MEDIC_MEAS_STATE");
        return MEDIC_MEAS_STATE;
    }

    return IMP_MEAS_STATE;
}

#if 0
static data_acq_state_t battery_meas_state_hanlder(void)
{
    if (!battery_is_charging())
    {
        medic_meas_start();
        return MEDIC_MEAS_STATE;
    }

    return BATTERY_MEAS_STATE;
}
#endif

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void data_acq_task(void)
{
    if (mem_dump_in_progress())
    {
        return;
    }

    switch (m_next_state)
    {
        case IDLE_STATE:
            m_next_state = idle_state_hanlder();
            break;

        case MEDIC_MEAS_STATE:
            m_next_state = medic_meas_state_hanlder();
            break;

        case IMP_MEAS_STATE:
            m_next_state = imp_meas_state_hanlder();
            break;

        #if 0
            case BATTERY_MEAS_STATE:
                m_next_state = battery_meas_state_hanlder();
                break;
        #endif

        default:
            break;
    }
}
