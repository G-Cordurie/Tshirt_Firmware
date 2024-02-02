#include "spi_flash.h"

static const nrf_drv_spi_t m_spi_master = NRF_DRV_SPI_INSTANCE(2);

static uint8_t m_tx_data[SPI_TX_RX_SIZE]; // A buffer with data to transfer.
static uint8_t m_rx_data[SPI_TX_RX_SIZE]; // A buffer for incoming data.

/**
 * @brief Internal SPI transfer for flash memory. Big data chunk accepted
 *
 * @param p_tx_data
 * @param tx_size
 * @param p_rx_data
 * @param rx_size
 * @return int32_t
 */
static int32_t flash_send(uint8_t *p_tx_data, uint16_t tx_size, uint8_t *p_rx_data, uint16_t rx_size)
{
    ret_code_t ret_code;

    int      idx     = 0;
    uint16_t left_tx = tx_size;
    uint16_t send_tx = 0;
    uint16_t left_rx = rx_size;
    uint16_t send_rx = 0;

    nrf_gpio_pin_clear(NCS_FLASH);

    while (left_tx != 0 || left_rx != 0)
    {
        // SPI max payload is 255. Must create a loop for longer tx_data or rx_data
        if (left_tx > PAYLOAD_SIZE)
        {
            send_tx = PAYLOAD_SIZE;
            left_tx -= PAYLOAD_SIZE;
        }
        else
        {
            send_tx = left_tx;
            left_tx = 0;
        }

        if (left_rx > PAYLOAD_SIZE)
        {
            send_rx = PAYLOAD_SIZE;
            left_rx -= PAYLOAD_SIZE;
        }
        else
        {
            send_rx = left_rx;
            left_rx = 0;
        }

        ret_code = nrf_drv_spi_transfer(&m_spi_master, &p_tx_data[idx], send_tx, &p_rx_data[idx], send_rx);
        if (ret_code != NRF_SUCCESS)
        {
            // TODO Manage Error
            INFO("[%s] error %d", __func__, ret_code);
            return NRF_ERROR_INTERNAL;
        }

        idx += PAYLOAD_SIZE;
    }

    nrf_gpio_pin_set(NCS_FLASH);

    return NRF_SUCCESS;
}

/**
 * @brief Get Operation in progress bit in status register
 *
 * @return uint8_t
 */
static uint8_t get_oip(void)
{
    memset(&m_tx_data, 0, sizeof(m_tx_data));

    m_tx_data[0] = OP_GET_FEATURE;
    m_tx_data[1] = OP_STATUS_REG;
    flash_send(m_tx_data, 2, m_rx_data, 3);

    return (m_rx_data[2] & 0x01) == 1;
}

/**
 * @brief Get Erase fail bit in status register
 *
 * @return uint8_t
 */
static uint8_t get_erase_fail(void)
{
    memset(&m_tx_data, 0, sizeof(m_tx_data));

    m_tx_data[0] = OP_GET_FEATURE;
    m_tx_data[1] = OP_STATUS_REG;
    flash_send(m_tx_data, 2, m_rx_data, 3);

    return (m_rx_data[2] & 0x04) == 4;
}

/**
 * @brief Get Program fail bit in status register
 *
 * @return uint8_t
 */
static uint8_t get_prgm_fail(void)
{
    memset(&m_tx_data, 0, sizeof(m_tx_data));

    m_tx_data[0] = OP_GET_FEATURE;
    m_tx_data[1] = OP_STATUS_REG;
    flash_send(m_tx_data, 2, m_rx_data, 3);

    return (m_rx_data[2] & 0x08) == 8;
}

static void wait_or_timeout(void)
{
    volatile uint32_t wait = FLASH_TIMEOUT;

    while (get_oip() && (wait != 0))
    {
        wait--;
    }

    if (wait == 0)
    {
        INFO("Function: %s, Flash memory timeout error!", (uint32_t) __func__);
    }
}

/**
 * @brief Do a software reset of the flash memory
 *
 */
static void flash_reset(void)
{
    memset(&m_tx_data, 0, sizeof(m_tx_data));

    m_tx_data[0] = OP_RESET;
    flash_send(m_tx_data, 1, NULL, 0);

    wait_or_timeout();
}

/**
 * @brief Enable Writing into flash memory
 *
 * @return int32_t
 */
static int32_t flash_write_enable(void)
{
    memset(&m_tx_data, 0, sizeof(m_tx_data));

    m_tx_data[0] = OP_WRITE_EN;
    if (flash_send(m_tx_data, 1, NULL, 0) != NRF_SUCCESS)
    {
        return NRF_ERROR_INTERNAL;
    }

    return NRF_SUCCESS;
}

/**
 * @brief Set value of the Block Lock register
 *
 * @param val
 */
static void flash_block_lock(uint8_t val)
{
    memset(&m_tx_data, 0, sizeof(m_tx_data));

    m_tx_data[0] = OP_SET_FEATURE;
    m_tx_data[1] = OP_BLOCK_LOCK;
    m_tx_data[2] = val;
    flash_send(m_tx_data, 3, NULL, 0);
}

/**
 * @brief Scan every blocks (page % 64) using page read command and check the data
 * at the column address 2048 of page 0 and page 1
 * If the read data is not 0xFF, the block is interpreted as an invalid block
 *
 * @param addr
 * @return true
 * @return false
 */
bool block_is_invalid(const uint32_t addr)
{
    // Loop for page 0 & 1
    for (uint32_t i = 0; i < 2; i++)
    {
        memset(&m_tx_data, 0, sizeof(m_tx_data));
        memset(&m_rx_data, 0, sizeof(m_rx_data));

        // OP + 8 dummy bits & 16 bits addr
        m_tx_data[0] = OP_PAGE_READ;
        m_tx_data[1] = ((addr + i) >> 16) & 0xFF;
        m_tx_data[2] = ((addr + i) >> 8) & 0xFF;
        m_tx_data[3] = (addr + i) & 0xFF;
        flash_send(m_tx_data, 4, NULL, 0);

        wait_or_timeout();

        // OP + 4 dummy bits & 12 bits addr
        m_tx_data[0] = OP_CACHE_READ;
        m_tx_data[1] = (2048 >> 8) & 0xFF;
        m_tx_data[2] = (2048) & 0xFF;
        flash_send(m_tx_data, 3, m_rx_data, 4 + 1);

        // Skip the first 4 empty bytes
        if (m_rx_data[4] != 0xFF)
        {
            return true;
        }
    }

    return false;
}

void flash_spi_init(void)
{
    nrf_drv_spi_config_t const config = {
        .sck_pin      = CK_FLASH,
        .mosi_pin     = SDI_FLASH,
        .miso_pin     = SDO_FLASH,
        .ss_pin       = WP_FLASH,
        .irq_priority = APP_IRQ_PRIORITY_LOW,
        .orc          = 0xFF,
        .frequency    = NRF_DRV_SPI_FREQ_8M,
        .mode         = NRF_DRV_SPI_MODE_3,
        .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
    };

    ret_code_t ret_code = nrf_drv_spi_init(&m_spi_master, &config, NULL, NULL);
    if (ret_code != NRF_SUCCESS)
    {
        INFO("[%s] error %d", __func__, ret_code); // TODO Manage Error
    }

    APP_ERROR_CHECK(ret_code);

    nrf_gpio_cfg_output(NCS_FLASH);
    nrf_gpio_pin_set(NCS_FLASH);
}

void flash_spi_uninit(void)
{
    nrf_drv_spi_uninit(&m_spi_master);
}

void flash_init(void)
{
    flash_reset();
    flash_block_lock(0x00);
}

void flash_uninit(void)
{
    flash_spi_uninit();
}

int32_t flash_erase_block(const uint32_t addr)
{
    if (addr < FLASH_MIN_ADDR || addr > FLASH_MAX_ADDR)
    {
        INFO("[%s] Wrong addr", __func__);
        return 1;
    }

    if (flash_write_enable() != NRF_SUCCESS)
        return NRF_ERROR_INTERNAL;

    memset(&m_tx_data, 0, sizeof(m_tx_data));

    m_tx_data[0] = OP_BLOCK_ERASE;
    m_tx_data[1] = (addr >> 16) & 0xFF;
    m_tx_data[2] = (addr >> 8) & 0xFF;
    m_tx_data[3] = (addr)&0xFF;
    if (flash_send(m_tx_data, 4, NULL, 0) != NRF_SUCCESS)
    {
        return NRF_ERROR_INTERNAL;
    }

    wait_or_timeout();

    if (get_erase_fail())
    {
        INFO("[%s] Erase program error", __func__);
        return NRF_ERROR_BUSY;
    }

    return NRF_SUCCESS;
}

uint8_t flash_read_manufacturer_id()
{
    memset(&m_tx_data, 0, sizeof(m_tx_data));

    m_tx_data[0] = OP_READ_ID;
    m_tx_data[1] = 0;
    flash_send(m_tx_data, 2, m_rx_data, 4);

    return m_rx_data[2];
}

uint8_t flash_read_device_id()
{
    memset(&m_tx_data, 0, sizeof(m_tx_data));

    m_tx_data[0] = OP_READ_ID;
    m_tx_data[1] = 0;
    flash_send(m_tx_data, 2, m_rx_data, 4);

    return m_rx_data[3];
}

int32_t flash_page_write(uint32_t const addr, uint8_t *in_buff, uint16_t size)
{
    if (size > FLASH_MAX_DATA_SIZE)
    {
        INFO("[%s] size too large (%d)", __func__, size);
        return NRF_ERROR_DATA_SIZE;
    }

    if (addr < FLASH_MIN_ADDR || addr > FLASH_MAX_ADDR)
    {
        INFO("[%s] Wrong addr", __func__);
        return NRF_ERROR_INVALID_ADDR;
    }

    // TODO check page defect
    if (flash_write_enable() != NRF_SUCCESS)
    {
        return NRF_ERROR_INTERNAL;
    }

    memset(&m_tx_data, 0, sizeof(m_tx_data));

    // OP + 4 dummy bits & 12 bits addr
    m_tx_data[0] = OP_PRGM_LOAD;
    m_tx_data[1] = 0;
    m_tx_data[2] = 0;
    memcpy(&m_tx_data[3], in_buff, size);
    if (flash_send(m_tx_data, 3 + size, m_rx_data, 0) != NRF_SUCCESS)
    {
        return NRF_ERROR_INTERNAL;
    }

    // OP + 8 dummy bits & 16 bits addr
    m_tx_data[0] = OP_PRGM_EXEC;
    m_tx_data[1] = (addr >> 16) & 0xFF;
    m_tx_data[2] = (addr >> 8) & 0xFF;
    m_tx_data[3] = (addr)&0xFF;
    if (flash_send(m_tx_data, 4, NULL, 0) != NRF_SUCCESS)
    {
        return NRF_ERROR_INTERNAL;
    }

    wait_or_timeout();

    if (get_prgm_fail())
    {
        INFO("[%s] Page write error", __func__);
        return NRF_ERROR_BUSY;
    }

    return NRF_SUCCESS;
}

int32_t flash_page_read(const uint32_t addr, uint8_t *out_buff, uint16_t size)
{
    if (size > FLASH_MAX_DATA_SIZE)
    {
        INFO("[%s] size too large (%d)", __func__, size);
        return NRF_ERROR_DATA_SIZE;
    }

    if (addr < FLASH_MIN_ADDR || addr > FLASH_MAX_ADDR)
    {
        INFO("[%s] Wrong addr", __func__);
        return NRF_ERROR_INVALID_ADDR;
    }

    memset(&m_tx_data, 0, sizeof(m_tx_data));
    memset(&m_rx_data, 0, sizeof(m_rx_data));

    // OP + 8 dummy bits & 16 bits addr
    m_tx_data[0] = OP_PAGE_READ;
    m_tx_data[1] = (addr >> 16) & 0xFF;
    m_tx_data[2] = (addr >> 8) & 0xFF;
    m_tx_data[3] = (addr)&0xFF;
    if (flash_send(m_tx_data, 4, NULL, 0) != NRF_SUCCESS)
    {
        return NRF_ERROR_INTERNAL;
    }

    wait_or_timeout();

    // OP + 4 dummy bits & 12 bits addr
    m_tx_data[0] = OP_CACHE_READ;
    m_tx_data[1] = 0;
    m_tx_data[2] = 0;
    m_tx_data[3] = 0;
    if (flash_send(m_tx_data, 4, m_rx_data, size + 4) != NRF_SUCCESS)
    {
        return NRF_ERROR_INTERNAL;
    }

    // Skip the first 4 empty bytes
    memcpy(out_buff, &m_rx_data[4], size);

    return NRF_SUCCESS;
}
