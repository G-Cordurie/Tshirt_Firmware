#ifndef SPIM0_MNGR_H
#define SPIM0_MNGR_H

#include <stdint.h>
#include "nrf_drv_spi.h"

#define SPIM0_TX_RX_BUF_LEN 8

typedef void (*spim0_uninit_cb)(void);

ret_code_t spim0_init(nrf_drv_spi_config_t const *spi_config, spim0_uninit_cb cb);
void spim0_uninit(void);
ret_code_t spim0_write(uint8_t const *data, uint8_t len);
ret_code_t spim0_transfer(uint8_t const *tx_data, uint8_t *rx_data, uint8_t len);

#endif // SPIM0_MNGR_H