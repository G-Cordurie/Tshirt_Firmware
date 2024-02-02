#include "nrf_error.h"
#include "nrf_drv_spi.h"
#include "spim0_mngr.h"

static const nrf_drv_spi_t m_spim0 = NRF_DRV_SPI_INSTANCE(0);
static spim0_uninit_cb m_spim0_uninit = NULL;
static uint8_t m_rx_data[SPIM0_TX_RX_BUF_LEN] = {0};

ret_code_t spim0_init(nrf_drv_spi_config_t const *spi_config, spim0_uninit_cb cb)
{
    if (cb == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    if (m_spim0_uninit != NULL)
    {
        m_spim0_uninit();
    }

    m_spim0_uninit = cb;
    return nrf_drv_spi_init(&m_spim0, spi_config, NULL, NULL);
}

void spim0_uninit(void)
{
    nrf_drv_spi_uninit(&m_spim0);
}

ret_code_t spim0_write(uint8_t const *data, uint8_t len)
{
    return nrf_drv_spi_transfer(&m_spim0, data, len, m_rx_data, len);
}

ret_code_t spim0_transfer(uint8_t const *tx_data, uint8_t *rx_data, uint8_t len)
{
    return nrf_drv_spi_transfer(&m_spim0, tx_data, len, rx_data, len);
}