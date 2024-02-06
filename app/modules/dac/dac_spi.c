#include <stdbool.h>
#include <string.h>

#include "app_error.h"
#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "sdk_errors.h"

#include "boards.h"
#include "debug.h"

#define TX_RX_BUF_LENGTH 16U

static const nrf_drv_spi_t m_spi_master                = NRF_DRV_SPI_INSTANCE(1);
static volatile uint8_t    m_tx_data[TX_RX_BUF_LENGTH] = {0};   // A buffer with data to transfer.
static volatile uint8_t    m_rx_data[TX_RX_BUF_LENGTH] = {0};   // A buffer for incoming data.
static volatile bool       m_dac_spi_initialized       = false; // DAC SPI initialization status

void dac_spi_init(void)
{
    if (!m_dac_spi_initialized)
    {
        /*
        nrf_drv_spi_config_t const config = {
            .sck_pin      = CK_DAK,
            .mosi_pin     = SDIN_DAC,
            .miso_pin     = NRF_DRV_SPI_PIN_NOT_USED,
            .ss_pin       = NSYNC_DAC,
            .irq_priority = APP_IRQ_PRIORITY_LOW,
            //.orc          = 0xCC,
            .frequency    = NRF_DRV_SPI_FREQ_4M,
            .mode         = NRF_DRV_SPI_MODE_1,
            .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
        
        };
        

        ret_code_t err_code = nrf_drv_spi_init(&m_spi_master, &config, NULL, NULL);
        APP_ERROR_CHECK(err_code);
        */

        m_dac_spi_initialized = true;
        return;
    }

    INFO("[DAC_SPI] > SPI is already initialized!");
}

void dac_spi_uninit(void)
{
    if (m_dac_spi_initialized)
    {
        nrf_drv_spi_uninit(&m_spi_master);
        m_dac_spi_initialized = false;
        return;
    }

    INFO("[DAC_SPI] > SPI has not been initialized!");
}

ret_code_t dac_spi_transfer(uint8_t const *buf, uint16_t len)
{
    if (!m_dac_spi_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (buf == NULL || len == 0 || len > TX_RX_BUF_LENGTH)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memcpy((uint8_t *)m_tx_data, buf, len);
    memset((uint8_t *)m_rx_data, 0, TX_RX_BUF_LENGTH);
    return nrf_drv_spi_transfer(&m_spi_master, (uint8_t const *)m_tx_data, len, (uint8_t *)m_rx_data, len);
}
