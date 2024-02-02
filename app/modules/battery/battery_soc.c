#include <stdbool.h>
#include <stdint.h>

#include "arm_math.h"

#include "app_ble_svc_bas.h"
#include "ble_gatts.h"
#include "nrf_gpio.h"

#include "battery.h"
#include "battery_chrgr.h"
#include "battery_soc.h"
#include "debug.h"
#include "pl482636.h"
#include "pwr_mngr.h"
#include "timer.h"

typedef struct interp_instance_tag
{
    const uint8_t    size;
    const float64_t *data;
} interp_instance_t;

static const interp_instance_t dschrg_interp_inst = {.data = (const float64_t *)dschrg_lut, .size = DSCHRG_LUT_SIZE};
static const interp_instance_t chrg_interp_inst   = {.data = (const float64_t *)chrg_lut, .size = CHRG_LUT_SIZE};

static float64_t interpolate_segment(float64_t x, float64_t x0, float64_t y0, float64_t x1, float64_t y1)
{
    float64_t t;

    if (islessequal(x, x0))
    {
        return y0;
    }

    if (isgreaterequal(x, x1))
    {
        return y1;
    }

    t = x - x0;
    t /= x1 - x0;

    if (isfinite(t))
    {
        float64_t res = y0 + t * (y1 - y0);
        if (isfinite(res))
        {
            return res;
        }
    }

    return y1;
}

float64_t battery_percent_compute(chrgr_state_t chrgr_state, float64_t volt)
{
    interp_instance_t *interp_instance;

    if (chrgr_state == CHRGR_CONNECTED_STATE)
    {
        interp_instance = (interp_instance_t *)&chrg_interp_inst;
    }
    else
    {
        interp_instance = (interp_instance_t *)&dschrg_interp_inst;
    }

    if (isgreater(volt, interp_instance->data[interp_instance->size - 1]))
    {
        return interp_instance->data[(interp_instance->size * 2) - 1];
    }
    else if (isless(volt, interp_instance->data[0]))
    {
        return interp_instance->data[interp_instance->size];
    }

    for (uint8_t segment = 0; segment < interp_instance->size - 1; segment++)
    {
        if (islessequal(interp_instance->data[segment], volt) && isgreaterequal(interp_instance->data[segment + 1], volt))
        {
            return interpolate_segment(volt, interp_instance->data[segment], interp_instance->data[interp_instance->size + segment],
                                       interp_instance->data[segment + 1], interp_instance->data[interp_instance->size + segment + 1]);
        }
    }

    return interp_instance->data[(interp_instance->size * 2) - 1];
}