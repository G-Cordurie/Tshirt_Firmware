#include "app_util_platform.h"
#include "nordic_common.h"
#include "nrf_gpio.h"

#include "boards.h"
#include "dac_spi.h"
#include "debug.h"
#include "imp.h"

static volatile bool m_dac_initialized = false;

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void dac_init(void)
{
    if (!m_dac_initialized)
    {
        dac_spi_init();
        m_dac_initialized = true;
    }
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/
/**
 * @brief Stop the stimulation DAC used for impedance by setting his status to 0 in 1 1 mode (unconnected)
 *
 */
void dac_disconnect(void)
{
    uint8_t cmd[2];

    cmd[0] = STIM[0] & 0x00ff;
    cmd[1] = (STIM[0] & 0x3f00) >> 8;

    CRITICAL_REGION_ENTER();
    {
        if (m_dac_initialized)
        {
            UNUSED_RETURN_VALUE(dac_spi_transfer(&cmd[0], 2));
            dac_spi_uninit();
            nrf_gpio_cfg_output(NSYNC_DAC);
            nrf_gpio_pin_set(NSYNC_DAC);
            m_dac_initialized = false;
        }
    }
    CRITICAL_REGION_EXIT();
}