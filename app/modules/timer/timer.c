#include "app_timer.h"

#include "debug.h"

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void timer_init(void)
{
    static uint8_t initialized = 0;

    if (!initialized)
    {
        ret_code_t err_code = app_timer_init();
        APP_ERROR_CHECK(err_code);

        initialized = 1;
    }
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/
