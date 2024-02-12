#ifndef MAX30001_H
# define MAX30001_H

//#include "evt_ctx.h"
#include "max30001_spi.h"
#include "nrf_drv_gpiote.h"

#define MAX_ECG_RESO    18
#define DIAG_ECG_RESO   12
#define MAX_IMP_RESO    20

extern uint32_t rtor_index;

void max30001_spi_init(void);
void max30001_init(void);
void max30001_meas_init(void);
void max30001_meas_stop(void);

#endif /* !MAX30001_H */
