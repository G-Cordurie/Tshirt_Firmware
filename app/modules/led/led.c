#include "nrf_drv_gpiote.h"

#include "boards.h"
#include "debug.h"

#define LED0_ON_STATE  0
#define LED0_OFF_STATE 1

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/
//Vu
void led0_init(void)
{
    ret_code_t err_code;

    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(LED0_OFF_STATE);

    err_code = nrf_drv_gpiote_out_init(LED0, &out_config);
    APP_ERROR_CHECK(err_code);
}

/*--------------------------------------------------------------------------*/
/*          Fonction redefinie en led_off en release dans debug.h           */
/*--------------------------------------------------------------------------*/

void led0_on(void)
{
    nrf_drv_gpiote_out_clear(LED0);
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void led0_off(void)
{
    nrf_drv_gpiote_out_set(LED0);
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/
