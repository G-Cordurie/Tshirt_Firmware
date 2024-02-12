#include "max30001_spi.h"

#include "nrf_gpio.h"
static const nrf_drv_spi_t m_spi_master = NRF_DRV_SPI_INSTANCE(1);
static volatile uint8_t m_max30001_spi_initialized;

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void max30001_spi_init(void)
{
    if (m_max30001_spi_initialized)
        return;

    nrf_drv_spi_config_t const config =
    {
        .sck_pin = SCL_Z,
        .mosi_pin = SDI_Z,
        .miso_pin = SDO_Z,
        .ss_pin = CSB,
        .irq_priority = APP_IRQ_PRIORITY_HIGHEST,
        .orc = 0xFF,
        .frequency = NRF_DRV_SPI_FREQ_8M,
        .mode = NRF_DRV_SPI_MODE_0,
        .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
    };

    ret_code_t ret_code = nrf_drv_spi_init(&m_spi_master, &config, NULL, NULL);
    if (ret_code != NRF_SUCCESS)
        INFO("[%s] error %d\n", __func__, ret_code);
    APP_ERROR_CHECK(ret_code);
    nrf_gpio_cfg_output(CSB);
    nrf_gpio_pin_set(CSB);

    m_max30001_spi_initialized = 1;
    INFO("Max30001 SPI init!\n");
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

static int32_t max_spi_send (uint8_t *p_tx_data, uint16_t tx_size,
        uint8_t *p_rx_data, uint16_t rx_size)
{
    ret_code_t ret_code;

    nrf_gpio_pin_clear(CSB);
    ret_code = nrf_drv_spi_transfer(&m_spi_master,
            p_tx_data, tx_size,
            p_rx_data, rx_size);
    if (ret_code != NRF_SUCCESS)
    {
        INFO ("[%s] error %d\n", __func__, ret_code);
        return NRF_ERROR_INTERNAL;
    }
    nrf_gpio_pin_set(CSB);
    return NRF_SUCCESS;
}

int max30001_reg_write(MAX30001_REG_map_t addr, uint32_t data)
{
    uint8_t m_tx_data[4];

    m_tx_data[0] = (addr << 1) & 0xFF;
    m_tx_data[1] = (data >> 16) & 0xff;
    m_tx_data[2] = (data >> 8) & 0xff;
    m_tx_data[3] = (data ) & 0xff;
    if (max_spi_send(m_tx_data, 4, NULL, 0) != NRF_SUCCESS)
        return NRF_ERROR_INTERNAL;
    return NRF_SUCCESS;
}

int max30001_reg_read(MAX30001_REG_map_t addr, uint8_t *data, uint16_t rx_size)
{
    uint8_t m_tx_data[1];

    m_tx_data[0] = ((addr << 1) & 0xFF) | 1; // For Read, Or with 1

    if (max_spi_send(m_tx_data, 1, data, rx_size) != NRF_SUCCESS)
        return NRF_ERROR_INTERNAL;
    return NRF_SUCCESS;
}

uint8_t max30001_spi_initialized(void)
{
    return m_max30001_spi_initialized;
}
