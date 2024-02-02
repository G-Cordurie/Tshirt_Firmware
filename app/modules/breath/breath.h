#ifndef BREATH_H
#define BREATH_H

#include <stdbool.h>
#include <stdint.h>

#define BREATH_SAMPLING_PERIOD_MS 250U

#if 0
extern bool prefs_auto_disconnect;
extern bool device_was_connected;

void breath_init(void);
void breath_tmr_start(void);
void breath_tmr_stop(void);
void store_breath(int16_t adc_value);
void init_auto_disconnect(void);
void breath_on_ble_connect_callback(void);
void breath_on_ble_disconnect_callback(void);
#endif

#endif // BREATH_H