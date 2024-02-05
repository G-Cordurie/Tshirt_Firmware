#include <stdbool.h>
#include <stdint.h>

#include "app_util_platform.h"
#include "nrf_gpio.h"

#include "acc.h"
#include "ble_diagw.h"
#include "boards.h"
#include "data_acq.h"
#include "data_session.h"
#include "debug.h"
#include "spim0_mngr.h"
#include "storage.h"
#include "timer.h"
#include "utils.h"

static volatile uint8_t m_acc_spi_initialized;

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void acc_spi_init(void)
{
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

    if (!m_acc_spi_initialized)
    {
        nrf_gpio_pin_set(CS_ACCEL);
        nrf_gpio_cfg_output(CS_ACCEL);

        spi_config.sck_pin      = CK_ACCEL;
        spi_config.mosi_pin     = SDI_ACCEL;
        spi_config.miso_pin     = SDO_ACCEL;
        spi_config.ss_pin       = NRF_DRV_SPI_PIN_NOT_USED;
        spi_config.irq_priority = APP_IRQ_PRIORITY_LOW;
        spi_config.orc          = 0xCC;
        spi_config.frequency    = NRF_DRV_SPI_FREQ_1M;
        spi_config.mode         = NRF_DRV_SPI_MODE_3;
        spi_config.bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;

        ret_code_t err_code = spim0_init(&spi_config, acc_spi_uninit);
        APP_ERROR_CHECK(err_code);

        m_acc_spi_initialized = true;
    }
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void acc_spi_uninit(void)
{
    if (m_acc_spi_initialized)
    {
        nrf_gpio_pin_set(CS_ACCEL);
        spim0_uninit();
        m_acc_spi_initialized = false;
    }
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void acc_spi_write(const uint8_t *data, uint8_t len)
{
    ret_code_t err_code;

    if (!m_acc_spi_initialized)
    {
        acc_spi_init();
    }

    if ((data == NULL) || (len == 0) || (len > SPIM0_TX_RX_BUF_LEN))
    {
        INFO("[%s] Invalid parameters!", __func__);
        return;
    }

    nrf_gpio_pin_clear(CS_ACCEL);
    err_code = spim0_write(data, len);
    nrf_gpio_pin_set(CS_ACCEL);

    APP_ERROR_CHECK(err_code);
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void acc_spi_transfer(uint8_t const *tx_data, uint8_t *rx_data, uint8_t len)
{
    ret_code_t err_code;

    if (!m_acc_spi_initialized)
    {
        acc_spi_init();
    }

    if ((tx_data == NULL) || (rx_data == NULL) || (len == 0))
    {
        INFO("[%s] Invalid parameters!", __func__);
        return;
    }

    nrf_gpio_pin_clear(CS_ACCEL);
    err_code = spim0_transfer(tx_data, rx_data, len);
    nrf_gpio_pin_set(CS_ACCEL);

    APP_ERROR_CHECK(err_code);
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/
/**@brief Init the accelerometer with default value
 *
 */
void acc_init(void)
{
    uint8_t acc_cmd[ACC_CFG_CMD_LEN];

    // Configure power mode (50hz (b0100) and low power)
    acc_cmd[0] = 0x20;
    acc_cmd[1] = 0x47;
    acc_spi_write(&acc_cmd[0], ACC_CFG_CMD_LEN);

    // Configure INT1 for motion interrupt to wakeup bl652
    // CTRL_REG3 0x22
    acc_cmd[0] = 0x22;
    acc_cmd[1] = 0x40; // 0x40 Enable AOI1 on PIN1
    acc_spi_write(&acc_cmd[0], ACC_CFG_CMD_LEN);

    // CTRL_REG4 0x23
    acc_cmd[0] = 0x23;
    acc_cmd[1] = 0x20; // 0x20 8g Scale 00100000
    acc_spi_write(&acc_cmd[0], ACC_CFG_CMD_LEN);

    // INT1_CFG 0x30
    acc_cmd[0] = 0x30;
    acc_cmd[1] = 0x95; // 0x95  Enable XLIE or YLIE or ZLIE interrupt generation,
    acc_spi_write(&acc_cmd[0], ACC_CFG_CMD_LEN);

    // INT1_DURATION 0x33
    acc_cmd[0] = 0x33;
    acc_cmd[1] = 0x01; // 0x32  Duration = 50LSBs * (1/10Hz) = 5s.
    acc_spi_write(&acc_cmd[0], ACC_CFG_CMD_LEN);

    // INT1_THX 0x32
    acc_cmd[0] = 0x32;
    acc_cmd[1] = 0x16; // 1 LSb = 62 mg @ FS = 8 g
    acc_spi_write(&acc_cmd[0], ACC_CFG_CMD_LEN);

    // Activate FIFO Stream Mode FIFO_CTRL_REG (2Eh)
    acc_cmd[0] = 0x2E;
    acc_cmd[1] = 0x80; // b10 (Stream mode) b000000 (No interrupt or trigger)
    acc_spi_write(&acc_cmd[0], ACC_CFG_CMD_LEN);

    // Activate FIFO CTRL_REG5 (24h)
    acc_cmd[0] = 0x24;
    acc_cmd[1] = 0x40; // b0 (REBOOT Mode) b1 (FIFO_EN) b000000 (4d detection)
    acc_spi_write(&acc_cmd[0], ACC_CFG_CMD_LEN);
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

uint8_t acc_spi_initialized(void)
{
    return m_acc_spi_initialized;
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/
