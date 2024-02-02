#ifndef ECG_H
#define ECG_H

#include <stdint.h>

#define ECG_SAMPLING_PERIOD_MS 50U

typedef enum ecg_gain_tag
{
    ecg_gain_0 = 0,
    ecg_gain_1 = 1,
} ecg_gain_t;

void ecg_init(void);
void ecg_hw_filter_gain_set(ecg_gain_t gain);

#endif // ECG_H