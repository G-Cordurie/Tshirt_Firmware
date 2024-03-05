#include "adc_cb.h"
#include "acc.h"
#include "adc.h"
#include "battery.h"
#include "ble_diagw.h"
#include "breath.h"
#include "data_acq.h"
#include "data_session.h"
#include "ecg.h"
#include "imp.h"
#include "led.h"
#include "storage.h"
#include "temp.h"

#define IMP_PERIODE_MS            30000U

#define ECG_SAMPLES_CNT           10U
#define BREATH_SAMPLES_CNT        10U
#define TEMP_SAMPLING_CNT         50U

#define ECG_SAMPLES_BUFFER_LEN    3U * SAADC_MEDIC_CH_SAMPLES
#define BREATH_SAMPLES_BUFFER_LEN 3U * SAADC_MEDIC_CH_SAMPLES

#include "arm_math.h"

//volatile static uint16_t Buf_sin[200];
extern volatile uint8_t Max_ECG[6];

/*--------------------------------------------------------------------------*/
/*                  Tableau de test pour transmission                       */
/*--------------------------------------------------------------------------*/

void Init_Buf_sin(void)
{
    /*
    uint8_t i;

    for(i=0;i<200;i++)
    {
        Buf_sin[i] = 2048 + 100*sin(6.28*i/200);
    }
    */
}

/*--------------------------------------------------------------------------*/
/*      Exécuté toutes les 20 ms, soit 4 échantillons ECG                   */
/*--------------------------------------------------------------------------*/

static void saadc_medic_callback(nrf_drv_saadc_evt_t const *p_event)
{
/*  Le MAX30001 effectue maintenant la mesure de l'ECG
    // ECG
    volatile static nrf_saadc_value_t ecg_samples_buffer[ECG_SAMPLES_BUFFER_LEN] = {0};
    volatile static uint8_t           ecg_samples_cntr                           = 0;

    static uint8_t Cnt_buf = 0;         //Ajout GC

    adc_channel_buf_cpy((nrf_saadc_value_t *const)&ecg_samples_buffer[ecg_samples_cntr], p_event->data.done.p_buffer,
                        SAADC_MEDIC_CH_SAMPLES, SAADC_CH_0, SAADC_MEDIC_CH_NBR);
    
    for(uint8_t i = 0;i<4;i++)       //Ajout GC
    {
        ecg_samples_buffer[ecg_samples_cntr+i] = Buf_sin[Cnt_buf++];
        if(Cnt_buf == 200) Cnt_buf = 0;
    }
    
    ecg_samples_cntr += SAADC_MEDIC_CH_SAMPLES;

    if (ecg_samples_cntr >= ECG_SAMPLES_CNT)
    {
        uint8_t  ecg_ble_buffer[BLE_DIAGW_MAX_ECG_CHAR_LEN] = {0};
        uint64_t timestamp                                  = data_session_sample_timestamp_get(ecg_idx_type);
        memcpy((uint8_t *)&ecg_ble_buffer[0], (uint8_t *)&timestamp, sizeof(timestamp));

        for (uint8_t i = 0, p = TIMESTAMP_LEN; i < ECG_SAMPLES_CNT; i = i + 2)
        {
            ecg_ble_buffer[p++] = (((uint16_t)ecg_samples_buffer[i]) & 0x0ff0) >> 4;
            ecg_ble_buffer[p]   = (((uint16_t)ecg_samples_buffer[i]) & 0x000f) << 4;
            ecg_ble_buffer[p++] += (((uint16_t)ecg_samples_buffer[i + 1]) & 0x0f00) >> 8;
            ecg_ble_buffer[p++] = ((uint16_t)ecg_samples_buffer[i + 1]) & 0x00ff;
        }

        ring_store(ECG_TYPE, ecg_ble_buffer, sizeof(ecg_ble_buffer));

        if (ecg_samples_cntr >= ECG_SAMPLES_BUFFER_LEN)
        {
            ecg_samples_buffer[0] = ecg_samples_buffer[ECG_SAMPLES_CNT];
            ecg_samples_buffer[1] = ecg_samples_buffer[ECG_SAMPLES_CNT + 1];
        }

        ecg_samples_cntr -= ECG_SAMPLES_CNT;
        nrf_gpio_pin_toggle(LED0);
    }
*/



 // ECG
    static uint8_t ecg_cntr = 8;
    static uint8_t ecg_buf[26];
    
    for(int8_t i=0;i<6;i++)
    {
        ecg_buf[i+ecg_cntr] = Max_ECG[i];
    }

    ecg_cntr += 6;

    if (ecg_cntr >= 23)
    {
        uint64_t timestamp  = data_session_sample_timestamp_get(ecg_idx_type);
        memcpy((uint8_t *)&ecg_buf[0], (uint8_t *)&timestamp, sizeof(timestamp));

        ring_store(ECG_TYPE, ecg_buf, 23);

        if (ecg_cntr > 23)
        {
            ecg_buf[8] = ecg_buf[23];
            ecg_buf[9] = ecg_buf[24];
            ecg_buf[10] = ecg_buf[25];
        }

        ecg_cntr -= 15;
    }

    // Breath
    volatile static uint8_t           breath_ble_buffer[BLE_DIAGW_MAX_BREATH_CHAR_LEN]  = {0};
    volatile static uint8_t           breath_ble_buf_idx                                = TIMESTAMP_LEN;
    volatile static nrf_saadc_value_t breath0_samples_buffer[BREATH_SAMPLES_BUFFER_LEN] = {0};
    volatile static nrf_saadc_value_t breath1_samples_buffer[BREATH_SAMPLES_BUFFER_LEN] = {0};
    volatile static uint8_t           breath_samples_cntr                               = 0;

    // BREATH 0 + 1 : 1/10 rate ecg put five in a row
    adc_channel_buf_cpy((nrf_saadc_value_t *const)&breath0_samples_buffer[breath_samples_cntr], p_event->data.done.p_buffer,
                        SAADC_MEDIC_CH_SAMPLES, SAADC_CH_0, SAADC_MEDIC_CH_NBR);
    adc_channel_buf_cpy((nrf_saadc_value_t *const)&breath1_samples_buffer[breath_samples_cntr], p_event->data.done.p_buffer,
                        SAADC_MEDIC_CH_SAMPLES, SAADC_CH_1, SAADC_MEDIC_CH_NBR);
    breath_samples_cntr += SAADC_MEDIC_CH_SAMPLES;

    if (breath_samples_cntr >= BREATH_SAMPLES_CNT)
    {
        float64_t breath0 = adc_buffer_avg((nrf_saadc_value_t const *const)breath0_samples_buffer, BREATH_SAMPLES_CNT);
        float64_t breath1 = adc_buffer_avg((nrf_saadc_value_t const *const)breath1_samples_buffer, BREATH_SAMPLES_CNT);

        uint16_t adc_value                      = (uint16_t)round(breath0);
        breath_ble_buffer[breath_ble_buf_idx++] = (adc_value & 0x0ff0) >> 4;
        breath_ble_buffer[breath_ble_buf_idx]   = (adc_value & 0x000f) << 4;

        adc_value = (uint16_t)round(breath1);
        breath_ble_buffer[breath_ble_buf_idx++] += (adc_value & 0x0f00) >> 8;
        breath_ble_buffer[breath_ble_buf_idx++] = adc_value & 0x00ff;

        if (breath_samples_cntr >= BREATH_SAMPLES_BUFFER_LEN)
        {
            breath0_samples_buffer[0] = breath0_samples_buffer[BREATH_SAMPLES_CNT];
            breath0_samples_buffer[1] = breath0_samples_buffer[BREATH_SAMPLES_CNT + 1];
            breath1_samples_buffer[0] = breath1_samples_buffer[BREATH_SAMPLES_CNT];
            breath1_samples_buffer[1] = breath1_samples_buffer[BREATH_SAMPLES_CNT + 1];
        }

        breath_samples_cntr -= BREATH_SAMPLES_CNT;

        if (breath_ble_buf_idx >= (TIMESTAMP_LEN + BREATH_DATA_LEN - 1))
        {
            uint64_t timestamp = data_session_sample_timestamp_get(breath_idx_type);
            memcpy((uint8_t *)&breath_ble_buffer[0], (uint8_t *)&timestamp, sizeof(timestamp));
            ring_store(BREATH_TYPE, (uint8_t *)breath_ble_buffer, sizeof(breath_ble_buffer));
            breath_ble_buf_idx = TIMESTAMP_LEN;
        }
    }

    // Acc
    
    uint8_t res[7]     = {0};
    uint8_t command[7] = {0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0x00}; // Reading acc : X_L, X_H, Y_L, Y_H, Z_L, Z_H
    {
        acc_spi_transfer(&command[0], (uint8_t *)&res[0], 7);

        uint16_t x = (((res[2] << 8) | res[1]) + 32768) >> 4;
        uint16_t y = (((res[4] << 8) | res[3]) + 32768) >> 4;
        uint16_t z = (((res[6] << 8) | res[5]) + 32768) >> 4;

        uint8_t acc_ble_buffer[BLE_DIAGW_MAX_ACC_CHAR_LEN] = {0};

        acc_ble_buffer[TIMESTAMP_LEN + 0] = (x & 0x0ff0) >> 4;
        acc_ble_buffer[TIMESTAMP_LEN + 1] = (x & 0x000f) << 4;
        acc_ble_buffer[TIMESTAMP_LEN + 2] = (y & 0x0ff0) >> 4;
        acc_ble_buffer[TIMESTAMP_LEN + 3] = (y & 0x000f) << 4;
        acc_ble_buffer[TIMESTAMP_LEN + 4] = (z & 0x0ff0) >> 4;
        acc_ble_buffer[TIMESTAMP_LEN + 5] = (z & 0x000f) << 4;

        uint64_t timestamp = data_session_sample_timestamp_get(acc_idx_type);
        memcpy((uint8_t *)&acc_ble_buffer[0], (uint8_t *)&timestamp, sizeof(timestamp));
        ring_store(ACC_TYPE, acc_ble_buffer, sizeof(acc_ble_buffer));
    }

    volatile static uint8_t temp_sampling_cntr = 0;

    // TEMP : 1hz
    if (++temp_sampling_cntr == TEMP_SAMPLING_CNT)
    {
        temp_sampling_cntr = 0;
        nrf_saadc_value_t temp_samples_buffer[SAADC_MEDIC_CH_SAMPLES];

        adc_channel_buf_cpy(temp_samples_buffer, p_event->data.done.p_buffer, SAADC_MEDIC_CH_SAMPLES, SAADC_CH_2, SAADC_MEDIC_CH_NBR);
        int16_t temp0 = temp_convert_ntc_to_temp(
            temp_convert_adc_to_ntc(adc_buffer_avg((nrf_saadc_value_t const *const)temp_samples_buffer, SAADC_MEDIC_CH_SAMPLES)));

        adc_channel_buf_cpy(temp_samples_buffer, p_event->data.done.p_buffer, SAADC_MEDIC_CH_SAMPLES, SAADC_CH_3, SAADC_MEDIC_CH_NBR);
        int16_t temp1 = temp_convert_ntc_to_temp(
            temp_convert_adc_to_ntc(adc_buffer_avg((nrf_saadc_value_t const *const)temp_samples_buffer, SAADC_MEDIC_CH_SAMPLES)));

        uint8_t  temp_ble_buffer[BLE_DIAGW_MAX_TEMP_CHAR_LEN] = {0};
        uint64_t timestamp                                    = data_session_sample_timestamp_get(temp_idx_type);
        memcpy((uint8_t *)&temp_ble_buffer[0], (uint8_t *)&timestamp, sizeof(timestamp));

        temp_ble_buffer[TIMESTAMP_LEN]     = (temp0 & 0xff00) >> 8;
        temp_ble_buffer[TIMESTAMP_LEN + 1] = (temp0 & 0x00ff);
        temp_ble_buffer[TIMESTAMP_LEN + 2] = (temp1 & 0xff00) >> 8;
        temp_ble_buffer[TIMESTAMP_LEN + 3] = (temp1 & 0x00ff);

        ring_store(TEMP_TYPE, temp_ble_buffer, sizeof(temp_ble_buffer));

        /*
        uint8_t res[7]     = {0};
        uint8_t command[7] = {0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0x00}; // Reading acc : X_L, X_H, Y_L, Y_H, Z_L, Z_H
        */
        //lecture d'accéléro pour purger un éventuel retard de lecture
        acc_spi_transfer(&command[0], (uint8_t *)&res[0], 7);

    }

    // Battery
    {
        nrf_saadc_value_t battery_samples_buffer[SAADC_MEDIC_CH_SAMPLES];
        adc_channel_buf_cpy(battery_samples_buffer, p_event->data.done.p_buffer, SAADC_MEDIC_CH_SAMPLES, SAADC_CH_4, SAADC_MEDIC_CH_NBR);
        battery_voltage_update(adc_buffer_avg((nrf_saadc_value_t const *const)battery_samples_buffer, SAADC_MEDIC_CH_SAMPLES));
    }

    
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

/**@brief SAADC Callback
 *
 * @details Read adc values on all channels
 * encode and send values to BLE stack if connected else store them
 */
void saadc_callback(nrf_drv_saadc_evt_t const *p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        if (p_event->data.done.size == SAADC_MEDIC_BUF_LEN)
        {
            if (imp_meas_state_get() == IMP_STANDBY)
            {
                ret_code_t err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_MEDIC_BUF_LEN);
                APP_ERROR_CHECK(err_code);

                saadc_medic_callback(p_event);
            }
        }
    }
}
