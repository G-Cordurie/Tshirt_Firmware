#ifndef Euro_Body_H
#define Euro_Body_H

//#define HW_REV_STR "0102"

// LED : vu
#define LED0 26U

// Battery : vu
#define VBAT NRF_SAADC_INPUT_AIN6
#define NCHG 16     //29U : signal routé sur 29, redirigé temporairement sur 16 qui est libre sur le composant
#define EN_SW 255U

// Temperature : vu
#define CMDE_OFF 19U
#define TEMP0 NRF_SAADC_INPUT_AIN7
#define TEMP1 NRF_SAADC_INPUT_AIN2

//Controle : vu
#define TS_CTRL 23U

// Mux
#define SEL_DIRECT 10U
#define SEL_DRIVE 11U

// Breath : vu
#define RESPI_UC_0 NRF_SAADC_INPUT_AIN0
#define RESPI_UC_1 NRF_SAADC_INPUT_AIN1

// MAX30001 : vu
#define CSB 24U
#define SCL_Z 22U
#define SDI_Z 20U
#define SDO_Z 17U
#define INTB 8U
#define INT2B 13U

#define ECG_IN NRF_SAADC_INPUT_AIN5     //Ajout temporaire, le,temps de basculer sur le MAX30001


// Accelerometer : vu
#define CS_ACCEL 14U
#define CK_ACCEL 9U
#define SDI_ACCEL 28U
#define SDO_ACCEL 12U
#define INT2 10U

// Flash : vu
#define NCS_FLASH 15U
#define CK_FLASH 6U
#define SDI_FLASH 5U
#define SDO_FLASH 27U
#define WP_FLASH 255U // TODO: to delete

#endif // Euro_Body_H

















