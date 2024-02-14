#include "ble_diagw.h"
#include "data_session.h"
#include "max30001.h"
#include "storage.h"

#include "arm_math.h"
#include "ecg.h"

uint32_t rtor_index = 0;

static volatile uint8_t allow_interrupts;

volatile uint8_t Max_ECG[6];

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

uint32_t max30001_get_dev_id(void)
{
    uint8_t id[4];

    // Due to internal initialization procedures, this command will not read-back valid data if it
    // is the first command executed following either a power-cycle event, or a SW_RST event.
    max30001_reg_read(INFO, id, sizeof (id));
    max30001_reg_read(INFO, id, sizeof (id));
    if (id[1] == 0x52 && id[2] == 0x10)
    {
        INFO ("MAX30001|Device ID found\n");
        return NRF_SUCCESS;
    }
    INFO ("MAX30001 not detected!\n");
    return NRF_ERROR_INTERNAL;
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

static int32_t complt_2(int32_t adc_value, uint8_t bit_resolution)
{
    // Get most significant bit to detect negative value
    if (adc_value & (1 << (bit_resolution - 1)))
        // Inversion of the 12 most significant bits in order to get a signed value
        return adc_value | (((uint32_t)pow(2, 32 - bit_resolution) - 1) << bit_resolution);
    return adc_value;
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

static int32_t center_ecg_data(int32_t adc_value, uint8_t center_resolution)
{
    int32_t mid = (1 << (center_resolution - 1)) - 1; // Obtaining the middle of the value range
    int32_t ret = 0x0;

    ret =  complt_2(adc_value, MAX_ECG_RESO);

    //To clearly detect saturation
    if (ret > mid)
        ret =  0xFFFFFFFF;
    else if (ret < -mid)
        ret = 0x00000000;
    else
        ret = mid + ret;

    return ret;
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

static int max30001_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    static uint8_t fifo_init = false;
    uint8_t ptag, etag;

    uint8_t status[4];
    max30001_reg_read(STATUS, status, sizeof (status));
    if (!fifo_init)
    {
        max30001_reg_write(FIFO_RST, 0x000000);
        fifo_init = true;
        return 2;
    }

    if (status[1] & 0x80) // Status INT for ECG
    {
        uint16_t size_ecg = (4 * 3) + 1; // From EFIT in MNGR_INT
        uint8_t ecg_data[size_ecg];
        //uint8_t ecg[BLE_DIAGW_MAX_ECG_CHAR_LEN];

        max30001_reg_read(ECG_FIFO_BURST, ecg_data, sizeof (ecg_data));

        //memcpy(ecg, &ecg_index, sizeof(ecg_index));
        //for (uint8_t i = 1, k = TIMESTAMP_LEN; i < size_ecg; i = i + 6)
        for (uint8_t i = 1, k = 0; i < size_ecg; i = i + 6)
        {
            // Check if it is valid ECG data
            ptag = (ecg_data[i + 2] & 0x07) + (ecg_data[i + 5] & 0x07);
            etag = ((ecg_data[i + 2] & 0x38) >> 3) + ((ecg_data[i + 5] & 0x38) >> 3);
            if (ptag == 0x0E && (etag == 0x00 || etag == 0x02))
            {
                int32_t data1  = (int32_t)((ecg_data[i + 0] << 16) + (ecg_data[i + 1] << 8)
                        + ecg_data[i + 2]);
                int32_t data2  = (int32_t)((ecg_data[i + 3] << 16) + (ecg_data[i + 4] << 8)
                        + ecg_data[i + 5]);
                data1 = center_ecg_data((data1 & 0xFFFFFFC0) >> 6, DIAG_ECG_RESO);
                data2 = center_ecg_data((data2 & 0xFFFFFFC0) >> 6, DIAG_ECG_RESO);

                //TODO Full resolution (only 12bits out of 18bits are used here)
                Max_ECG[k++] =  (data1 & 0x0FF0) >> 4;
                Max_ECG[k] =    (data1 & 0x000F) << 4;

                Max_ECG[k++] += (data2 & 0x0F00) >> 8;
                Max_ECG[k++] =   data2 & 0x00FF;
            }
            else
            {
                INFO ("Error on FIFO ECG (p:%X, e:%x)\n", ptag, etag);
            }
        }

/*
        uint64_t timestamp = data_session_sample_timestamp_get(ecg_idx_type);
        memcpy(ecg, (uint8_t *)&timestamp, sizeof(timestamp));
        ring_store(ECG_TYPE, ecg, sizeof(ecg));
*/
        nrf_gpio_pin_toggle(LED0);
        
        //RING_STORE(ECG_TYPE, ecg);
        //ecg_index++;
    }

    // No Bioz in Europrotect
    //if (status[1] & 0x08) // Status INT for Bioz
    //{
    //    uint8_t btag;
    //    uint16_t size_imp = (4 * 3) + 1; // From BFIN in MNGR_INT
    //    uint8_t imp_data[size_imp];
    //    uint8_t imp[BLE_DIAGW_MAX_IMP_CHAR_LEN];

    //    max30001_reg_read(BIOZ_FIFO_BURST, imp_data, sizeof (imp_data));

    //    memcpy(imp, &imp_index, sizeof(imp_index));
    //    for (uint8_t i = 1, k = sizeof(imp_index); i < size_imp; i = i + 3)
    //    {
    //        btag = imp_data[i + 2] & 0x07;
    //        if (btag == 0x00 || btag == 0x02) // Check if it is a valid Bioz data
    //        {
    //            uint32_t raw_adc_value  = (int32_t)((imp_data[i + 0] << 16) + (imp_data[i + 1] << 8)
    //                    + imp_data[i + 2]);
    //            int32_t adc_value = complt_2((raw_adc_value & 0xFFFFFFF0) >> 4, MAX_IMP_RESO);
    //            int32_t ohm = adc_value / (pow(2, 19) * (32 * pow(10, -6)) * 10);
    //            if (ohm < 0)
    //                ohm = 0;
    //            //TODO Try to correct max30001 approx error
    //            //ohm += ohm * 0.22;

    //            //TODO Full resolution (only 16bits out of 20bits are used here)
    //            imp[k++] = (ohm & 0xFF00) >> 8;
    //            imp[k++] = (ohm & 0x00FF);
    //        }
    //        else
    //        {
    //            INFO ("Error on FIFO IMP (b:%x)\n", btag);
    //        }
    //    }
    //    RING_STORE(IMP_TYPE, imp);
    //    imp_index++;
    //}

/*      Partie envoi du rtor desactivee pour le moment (GC)
    if (status [2] & 0x04) // Status INT for R-to-R
    {
       #define SIZE_AVG 10
        uint16_t size_rtor = (1 * 3) + 1;
        uint8_t rtor_data[size_rtor];
        uint8_t rtor[BLE_DIAGW_MAX_IMP_CHAR_LEN];

        static uint32_t avg_rtor[SIZE_AVG];

        max30001_reg_read(RTOR, rtor_data, sizeof (rtor_data));
        uint32_t raw_adc_value = 8 * (((rtor_data[1] << 16) + (rtor_data[2] << 8) + (rtor_data[3])) >> 10);


        avg_rtor[rtor_index % SIZE_AVG] = raw_adc_value;
        if (rtor_index >= SIZE_AVG)
        {
            uint32_t sum_rtor = 0;
            for (uint8_t i = 0; i < SIZE_AVG; i++)
            {
                sum_rtor += avg_rtor[i];
            }

            sum_rtor = sum_rtor / SIZE_AVG;

            memcpy(rtor, &rtor_index, sizeof(rtor_index));
            uint8_t k = sizeof(rtor_index);
            rtor[k++] = (sum_rtor & 0xFF00) >> 8;
            rtor[k++] = (sum_rtor & 0x00FF);
            rtor[k++] = (sum_rtor & 0xFF00) >> 8;
            rtor[k++] = (sum_rtor & 0x00FF);
            rtor[k++] = (60000 / sum_rtor & 0xFF00) >> 8;
            rtor[k++] = (60000 / sum_rtor & 0x00FF);
            rtor[k++] = (60000 / sum_rtor & 0xFF00) >> 8;
            rtor[k++] = (60000 / sum_rtor & 0x00FF);

            RING_STORE(IMP_TYPE, rtor);
        }
        rtor_index++;
    }
    */
    return 0;
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void max30001_intB_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (allow_interrupts == false)
        return;
    max30001_int_handler(pin, action);
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void max30001_int2B_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (allow_interrupts == false)
        return;
    max30001_int_handler(pin, action);
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void max30001_init(void)
{
    ret_code_t err_code;
    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

    err_code = max30001_get_dev_id();
    APP_ERROR_CHECK(err_code);

// No Mux in Europrotect
//#if (HW_REV == HW_REV_0101)
//    nrf_gpio_cfg_output(SEL_DIRECT);
//    nrf_gpio_cfg_output(SEL_DRIVE);
//    nrf_gpio_pin_clear(SEL_DIRECT);
//    nrf_gpio_pin_clear(SEL_DRIVE);
//#elif (HW_REV == HW_REV_0102)
//    nrf_gpio_cfg_output(SEL_N);
//    nrf_gpio_cfg_output(SEL_P);
//    nrf_gpio_cfg_output(SEL_Z);
//    nrf_gpio_pin_clear(SEL_Z);
//    nrf_gpio_pin_clear(SEL_N);
//    nrf_gpio_pin_clear(SEL_P); // REF ON if (SEL_P == 0)
//#endif

    INFO ("Max30001 Init Done.\n");
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void max30001_meas_init(void)
{
    ret_code_t err_code;
    nrf_drv_gpiote_in_config_t cfg = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    cfg.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(INTB, &cfg, max30001_intB_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_in_init(INT2B, &cfg, max30001_int2B_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(INTB, true);
    nrf_drv_gpiote_in_event_enable(INT2B, true);

    max30001_reg_write(EN_INT,      0x000003);
    max30001_reg_write(EN_INT2,     0x000003);

    max30001_reg_write(CNFG_EMUX,   0x000000);
    max30001_reg_write(CNFG_BMUX,   0x000040);

    max30001_reg_write(CNFG_ECG,    0x805000);      //200 sps
    //max30001_reg_write(CNFG_BIOZ,   0x201130);
    max30001_reg_write(CNFG_RTOR1,   0x3FA300);
    max30001_reg_write(CNFG_RTOR2,   0x404800);

    max30001_reg_write(CNFG_GEN,    0x280013);      //fMSTR = 32000Hz

    max30001_reg_write(MNGR_INT,    0x1B0004);      //(1B -> 4 ou 4B -> 10) valeurs dans la fifo -> IT

    //max30001_reg_write(EN_INT,      0x880403);      //Int fifo en, Bioz en, RRint en 
    max30001_reg_write(EN_INT,      0x800003);      //Int fifo 
    max30001_reg_write(EN_INT2,     0x000303);      //Sample synchro
    max30001_reg_write(SYNCH,       0x000000);
    allow_interrupts = true;
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void max30001_meas_stop(void)
{
    max30001_reg_write(CNFG_GEN,    0x200004);
    max30001_reg_write(EN_INT,      0x000003);
    max30001_reg_write(EN_INT2,     0x000003);

    nrf_drv_gpiote_in_event_enable(INTB, true);
    nrf_drv_gpiote_in_event_enable(INT2B, true);

    max30001_reg_write(SW_RST,      0x000000);
    allow_interrupts = false;
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/
