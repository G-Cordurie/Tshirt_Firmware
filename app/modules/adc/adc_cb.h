#ifndef ADC_CB_H
#define ADC_CB_H

#include "nrf_drv_saadc.h"

void saadc_callback(nrf_drv_saadc_evt_t const *p_event);
void Init_Buf_sin(void);

#endif