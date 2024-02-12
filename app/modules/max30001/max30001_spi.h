#ifndef MAX30001_SPI_H
# define MAX30001_SPI_H

#include "boards.h"
#include "debug.h"
#include "nrf_drv_spi.h"

// MAX30001 Register addresses
typedef enum MAX30001_REG_map_tag
{
  STATUS =      0x01,
  EN_INT =      0x02,
  EN_INT2 =     0x03,
  MNGR_INT =    0x04,
  MNGR_DYN =    0x05,
  SW_RST =      0x08,
  SYNCH =       0x09,
  FIFO_RST =    0x0A,
  INFO =        0x0F,
  CNFG_GEN =    0x10,
  CNFG_CAL =    0x12,
  CNFG_EMUX =   0x14,
  CNFG_ECG =    0x15,
  CNFG_BMUX =   0x17,
  CNFG_BIOZ =   0x18,
  CNFG_PACE =   0x1A,
  CNFG_RTOR1 =  0x1D,
  CNFG_RTOR2 =  0x1E,

  // Data locations
  ECG_FIFO_BURST =  0x20,
  ECG_FIFO =        0x21,
  BIOZ_FIFO_BURST = 0x22,
  BIOZ_FIFO =       0x23,
  RTOR =            0x25,

  PACE0_FIFO_BURST =    0x30,
  PACE0_A =             0x31,
  PACE0_B =             0x32,
  PACE0_C =             0x33,

  PACE1_FIFO_BURST =    0x34,
  PACE1_A =             0x35,
  PACE1_B =             0x36,
  PACE1_C =             0x37,

  PACE2_FIFO_BURST =    0x38,
  PACE2_A =             0x39,
  PACE2_B =             0x3A,
  PACE2_C =             0x3B,

  PACE3_FIFO_BURST =    0x3C,
  PACE3_A =             0x3D,
  PACE3_B =             0x3E,
  PACE3_C =             0x3F,

  PACE4_FIFO_BURST =    0x40,
  PACE4_A =             0x41,
  PACE4_B =             0x42,
  PACE4_C =             0x43,

  PACE5_FIFO_BURST =    0x44,
  PACE5_A =             0x45,
  PACE5_B =             0x46,
  PACE5_C =             0x47,

} MAX30001_REG_map_t;

void max30001_spi_init(void);
int max30001_reg_write(MAX30001_REG_map_t addr, uint32_t data);
int max30001_reg_read(MAX30001_REG_map_t addr, uint8_t *data, uint16_t rx_size);
uint8_t max30001_spi_initialized(void);

#endif /* !MAX30001_SPI_H */
