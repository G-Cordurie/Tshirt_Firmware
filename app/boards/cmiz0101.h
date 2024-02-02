#ifndef CMIZ0101_H
#define CMIZ0101_H

#define HW_REV_STR "0101"

// LED
#define LED0 21U

// Battery
#define EN_SW 8U
#define VBAT NRF_SAADC_INPUT_AIN2
#define NCHG 9U

// Temperature
#define CMDE_OFF 7U
#define TEMP0 NRF_SAADC_INPUT_AIN1
#define TEMP1 NRF_SAADC_INPUT_AIN0

// Mux
#define SEL_DIRECT 10U
#define SEL_DRIVE 11U

// Breath
#define RESPI_UC_0 NRF_SAADC_INPUT_AIN7
#define RESPI_UC_1 NRF_SAADC_INPUT_AIN6

// MAX30001
// #define CSB 31U // 26U
// #define SCL_Z 26U // 16U
// #define SDI_Z 29U // 5U
// #define SDO_Z 30U // 18U
// #define INTB 12U
// #define INT2B 13U //14U

#define CSB 26U
#define SCL_Z 16U
#define SDI_Z 5U
#define SDO_Z 18U
#define INTB 12U
#define INT2B 14U

// SPO2
#define SPO2_0 19U
#define SPO2_1 29U
#define SPO2_2 23U
#define SPO2_3 28U

// Accelerometer
#define CS_ACCEL 20U
#define CK_ACCEL 17U
#define SDI_ACCEL 24U
#define SDO_ACCEL 22U
#define INT1 13U // INT1 is INT2 on acc, TODO: Replace INT2 by INT1 in app.

// Flash
#define NCS_FLASH 6U
#define CK_FLASH 27U
#define SDI_FLASH 25U
#define SDO_FLASH 15U
#define WP_FLASH 255U // TODO: to delete

#endif // CMIZ0101_H