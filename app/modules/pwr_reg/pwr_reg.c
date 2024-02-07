#include <stdint.h>

#include "nrf_delay.h"
#include "nrf_gpio.h"

#include "boards.h"
#include "debug.h"

#define REG_EN_STEPS        1000U
#define REG_STARTUP_TIME_MS 250U // Roughly estimated delay!

static volatile bool reg_enabled = false;

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/
/**
 * SStime (sec) = (Tstart-up at 0 pF) + ((0.6 × Css )/Iss)
 * Tstart-up at 0 pF = 380.0E-6
 * Css = 1.0E-9 F
 * Iss = 1.15E-6 A
 * SStime ≈ 1 ms
 * ENdelay-time ≈ 100 µs
 * MOSFET switching delay ≈ 1 µs
 */
void pwr_ldo_reg_enable(void)
{
    if (!reg_enabled)
    {
        nrf_gpio_pin_set(CMDE_OFF);       //Pour hard tshirt : nrf_gpio_pin_clear(CMDE_OFF);
        nrf_gpio_cfg_output(CMDE_OFF);

        for (uint16_t i = 0; i < REG_EN_STEPS; i++)
        {
            nrf_gpio_pin_set(CMDE_OFF);     //Tshirt : nrf_gpio_pin_clear(CMDE_OFF);
            nrf_delay_us(i);
            nrf_gpio_pin_clear(CMDE_OFF);   //Tshirt : nrf_gpio_pin_set(CMDE_OFF);
            nrf_delay_us(REG_EN_STEPS - i);
        }

        nrf_gpio_pin_set(CMDE_OFF);         //Tshirt : nrf_gpio_pin_clear(CMDE_OFF);

        nrf_gpio_pin_clear(Pol_N);         
        nrf_gpio_cfg_output(Pol_N);

        nrf_gpio_pin_set(Pol_P);         
        nrf_gpio_cfg_output(Pol_P);

        reg_enabled = true;
    }
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void pwr_ldo_reg_disable(void)
{
    nrf_gpio_pin_clear(CMDE_OFF);         //Pour hard tshirt : nrf_gpio_pin_set(CMDE_OFF);
    nrf_gpio_cfg_output(CMDE_OFF);
    nrf_delay_ms(REG_STARTUP_TIME_MS);

    nrf_gpio_pin_clear(Pol_N);         
    nrf_gpio_cfg_output(Pol_N);

    nrf_gpio_pin_clear(Pol_P);         
    nrf_gpio_cfg_output(Pol_P);

    reg_enabled = false;
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/
