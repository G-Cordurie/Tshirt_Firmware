#ifndef BATTERY_SOC_H
#define BATTERY_SOC_H

#include "arm_math.h"

#include "battery_chrgr.h"

#define BATTERY_CHARGED_THRESHOLD  ((float64_t)100.0)
#define BATTERY_SHUTDOWN_THRESHOLD ((float64_t)3.0) // 3.536 Volts (Discharging)
#define BATTERY_POWERUP_THRESHOLD  ((float64_t)5.0) // 3.670 Volts (Charging)

float64_t battery_percent_compute(chrgr_state_t chrgr_state, float64_t volt);

#endif // BATTERY_SOC_H