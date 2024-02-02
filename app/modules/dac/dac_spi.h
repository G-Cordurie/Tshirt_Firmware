#ifndef SPI_DAC_H
#define SPI_DAC_H

#include <stdint.h>

#include "sdk_errors.h"

void       dac_spi_init(void);
void       dac_spi_uninit(void);
ret_code_t dac_spi_transfer(uint8_t const *buf, uint16_t len);

#endif // SPI_DAC_H
