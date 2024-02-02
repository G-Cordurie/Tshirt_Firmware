#include "nrf_drv_clock.h"

void hfclk_clock_init(void)
{
    nrf_drv_clock_init(); // Enable external crystal for improved accuracy (Without this, the timers are drifting)
    nrf_drv_clock_hfclk_request(NULL);

    // TODO: change the while with a callback and add a timeout!
    while (!nrf_drv_clock_hfclk_is_running()) {}
}

void hfclk_clock_release(void)
{
    nrf_drv_clock_hfclk_release();

    // TODO: Add a timeout!
    // while (nrf_drv_clock_hfclk_is_running()) {}
}