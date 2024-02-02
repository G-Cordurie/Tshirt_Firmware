#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>

#include "arm_math.h"

void       battery_meas_start(void);
void       battery_switch_enable(void);
void       battery_switch_disable(void);
float64_t  battery_adc_to_raw_volt_convert(float64_t adc_val);
void       battery_voltage_update(float64_t adc_val);
float64_t  battery_voltage_get(void);
bool       battery_voltage_meas_started(void);
ret_code_t battery_safe_boot_check(void);

#endif // BATTERY_H
