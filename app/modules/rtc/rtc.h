#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include <stdint.h>

void     rtc_init(void);
void     rtc_uninit(void);
void     rtc_start(void);
void     rtc_stop(void);
uint64_t rtc_timestamp_get(void);

#endif // TIMESTAMP_H