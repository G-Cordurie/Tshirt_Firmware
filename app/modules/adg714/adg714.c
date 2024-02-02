#include <stdint.h>

#include "app_util_platform.h"
#include "nrf_gpio.h"

#include "adg714.h"
#include "boards.h"
#include "debug.h"
#include "spim0_mngr.h"

static volatile uint8_t m_adg714_spi_initialized;

void adg714_init(void)
{
    nrf_gpio_pin_set(NSYNC_UC);
    nrf_gpio_cfg_output(NSYNC_UC);

    nrf_gpio_pin_clear(SCK_UC);
    nrf_gpio_cfg_output(SCK_UC);

    nrf_gpio_pin_clear(DOUT_UC);
    nrf_gpio_cfg_output(DOUT_UC);
}

void adg714_spi_init(void)
{
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

    if (!m_adg714_spi_initialized)
    {
        nrf_gpio_pin_set(NSYNC_UC);
        nrf_gpio_cfg_output(NSYNC_UC);

        spi_config.sck_pin      = SCK_UC;
        spi_config.mosi_pin     = DOUT_UC;
        spi_config.miso_pin     = NRF_DRV_SPI_PIN_NOT_USED;
        spi_config.ss_pin       = NRF_DRV_SPI_PIN_NOT_USED;
        spi_config.irq_priority = APP_IRQ_PRIORITY_LOW;
        spi_config.orc          = 0xCC;
        spi_config.frequency    = NRF_DRV_SPI_FREQ_8M;
        spi_config.mode         = NRF_DRV_SPI_MODE_1;
        spi_config.bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;

        ret_code_t err_code = spim0_init(&spi_config, adg714_spi_uninit);
        APP_ERROR_CHECK(err_code);

        m_adg714_spi_initialized = 1;
    }
}

void adg714_spi_uninit(void)
{
    if (m_adg714_spi_initialized)
    {
        nrf_gpio_pin_set(NSYNC_UC);
        spim0_uninit();
        m_adg714_spi_initialized = false;
    }
}

void adg714_spi_write(uint8_t const *data, uint8_t len)
{
    if (!m_adg714_spi_initialized)
    {
        adg714_spi_init();
    }

    if ((data == NULL) || (len == 0) || (len > SPIM0_TX_RX_BUF_LEN))
    {
        INFO("[%s] Invalid parameters!", __func__);
        return;
    }

    nrf_gpio_pin_clear(NSYNC_UC);
    ret_code_t err_code = spim0_write(data, len);
    nrf_gpio_pin_set(NSYNC_UC);

    APP_ERROR_CHECK(err_code);
}