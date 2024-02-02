#ifndef ADC_H
#define ADC_H

#include "arm_math.h"
#include "nrf_drv_saadc.h"

#define SAADC_CH_0                0U
#define SAADC_CH_1                1U
#define SAADC_CH_2                2U
#define SAADC_CH_3                3U
#define SAADC_CH_4                4U
#define SAADC_CH_5                5U

#define SAADC_MEDIC_CH_NBR        6U
#define SAADC_MEDIC_CH_SAMPLES    4U
#define SAADC_MEDIC_BUF_LEN       (SAADC_MEDIC_CH_NBR * SAADC_MEDIC_CH_SAMPLES)

#define SAADC_BATTERY_CH_NBR      1U
#define SAADC_BATTERY_CH_SAMPLES  2U
#define SAADC_BATTERY_BUF_LEN     (SAADC_BATTERY_CH_NBR * SAADC_BATTERY_CH_SAMPLES)

#define SAADC_TEMP_CH_NBR         2U
#define SAADC_TEMP_CH_SAMPLES     10U
#define SAADC_TEMP_BUF_LEN        (SAADC_TEMP_CH_NBR * SAADC_TEMP_CH_SAMPLES)

#define SAADC_IMP_CH_NBR          1U
#define SAADC_IMP_CH_SAMPLES      1U
#define SAADC_IMP_BUF_LEN         (SAADC_IMP_CH_NBR * SAADC_IMP_CH_SAMPLES)

#define SAADC_BATTERY_SAMPLE_RATE 50000U
#define SAADC_MEDIC_SAMPLE_RATE   5000U
#define SAADC_TEMP_SAMPLE_RATE    5000U
#define SAADC_IMP_SAMPLE_RATE     50U

void adc_uninit(void);
void adc_medic_init(nrf_drv_saadc_event_handler_t event_handler);
void adc_battery_init(nrf_drv_saadc_event_handler_t event_handler);
void adc_imp_init(nrf_drv_saadc_event_handler_t event_handler);
void adc_temp_init(nrf_drv_saadc_event_handler_t event_handler);
void adc_channel_buf_cpy(nrf_saadc_value_t *const dest, nrf_saadc_value_t const *const src, uint16_t len, uint16_t offset, uint16_t step);
float64_t adc_buffer_avg(nrf_saadc_value_t const *const data, uint8_t len);

void adc_medic_meas_init(void);
void adc_meas_stop(void);

#endif // ADC_H