#include "nrf_gpio.h"

#include "boards.h"
#include "debug.h"
#include "ecg.h"

static volatile ecg_gain_t m_ecg_gain = ecg_gain_0;

static inline void ecg_switch_enable(void)
{
    nrf_gpio_cfg_output(SELECT);
    nrf_gpio_pin_set(SELECT);
}

static void ecg_hw_filter_init(void)
{
#if HW_TYPE == HW_TYPE_0706
    nrf_gpio_cfg_output(GAIN);

    if (m_ecg_gain == ecg_gain_0) // Select ECG filter gain
    {
        nrf_gpio_pin_clear(GAIN);
    }
    else
    {
        nrf_gpio_pin_set(GAIN);
    }
#endif

    ecg_switch_enable();
}

void ecg_init(void)
{
    ecg_hw_filter_init();
}

void ecg_hw_filter_gain_set(ecg_gain_t gain)
{
#if HW_TYPE == HW_TYPE_0706
    m_ecg_gain = gain;
    ecg_init();
#endif
}