#ifndef TEMP_H
#define TEMP_H

#include <stdbool.h>
#include <stdint.h>

#include "arm_math.h"

#define TEMP_SAMPLING_PERIOD_MS 1000U

bool      temp_calib_check(void);
void      temp_calib_run(void);
void      temp_calib_set_status(uint32_t status);
void      temp_system_reset(void);
float64_t temp_convert_adc_to_ntc(float64_t adc_val);
int16_t   temp_convert_ntc_to_temp(float64_t ntc_val);
float64_t temp_vcc_get(void);
float64_t temp_calib_coef_get(void);

#endif // TEMP_H