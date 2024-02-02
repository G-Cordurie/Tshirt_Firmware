#ifndef ACC_H
#define ACC_H

#include <stdint.h>

#define ACC_CFG_CMD_LEN        2U
#define ACC_SAMPLING_PERIOD_MS 20U

void    acc_init(void);
void    acc_spi_init(void);
void    acc_spi_uninit(void);
void    acc_spi_write(uint8_t const *data, uint8_t len);
void    acc_spi_transfer(uint8_t const *tx_data, uint8_t *rx_data, uint8_t len);
uint8_t acc_spi_initialized(void);

#endif // ACC_H
