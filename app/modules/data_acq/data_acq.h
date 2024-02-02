#ifndef EVT_CTX_H
#define EVT_CTX_H

#include <stdbool.h>

#include "adc_cb.h"
#include "ble_diagw.h"

void data_acq_task(void);
void medic_meas_start(void);
void medic_meas_stop(void);

#endif // EVT_CTX_H
