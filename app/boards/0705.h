#ifndef _0705_H
#define _0705_H

#define LED0       21
#define CMDE_OFF   7  /* 1 to cut - 0 to Power */
#define EN_SW      8  /* Alim Bat 1 to power - 0 to cut */
#define INT2       13 /* INT1 is INT2 on acc */
#define NCS_FLASH  6  /* Chip Select Flash*/
#define WP_FLASH   255
#define SDA_SPARE  12
#define SCL_SPARE  14
#define NSYNC_DAC  26
#define CK_DAK     16
#define CK_ACCEL   17
#define SDIN_DAC   18
#define CS_ACCEL   24
#define SDO_ACCEL  22
#define SCK_UC     11
#define NSYNC_UC   23
#define DOUT_UC    19
#define SDI_ACCEL  20
#define SDI_FLASH  25
#define CK_FLASH   27
#define SDO_FLASH  15
#define MES_ZTH    NRF_SAADC_INPUT_AIN5
#define RESPI_UC_0 NRF_SAADC_INPUT_AIN7
#define RESPI_UC_1 NRF_SAADC_INPUT_AIN6
#define ECG_IN     NRF_SAADC_INPUT_AIN3
#define TEMP0      NRF_SAADC_INPUT_AIN1
#define TEMP1      NRF_SAADC_INPUT_AIN0
#define VBAT       NRF_SAADC_INPUT_AIN2
#define SELECT     28

#endif // _0705_H