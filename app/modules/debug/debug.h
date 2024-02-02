#ifndef DEBUG_H
#define DEBUG_H

#include "app_config.h"
#include "led.h"

#ifdef DEBUG

#include "SEGGER_RTT.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define INFO(f_, ...) NRF_LOG_DEBUG((f_), ##__VA_ARGS__)
#define LED_OFF()     led0_off()
#define LED_ON()      led0_on()

#else

#define INFO(f_, ...)
#define LED_OFF() led0_off()
#define LED_ON()  LED_OFF() // LED is forced to OFF in release mode

#endif

void log_init(void);

#if APP_MODULE_ENABLED(APP_BLE_NUS)
void log_ble(__const char *__restrict __format, ...);
void log_tmr_start(void);
void log_tmr_stop(void);
#endif

#endif /* DEBUG_H */