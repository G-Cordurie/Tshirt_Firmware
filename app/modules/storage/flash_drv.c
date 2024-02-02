#include <stdint.h>

#include "app_util_platform.h"
#include "nordic_common.h"
#include "nrf_error.h"

#include "debug.h"
#include "flash_drv.h"
#include "spi_flash.h"

static flash_region_ctx_t m_data_region_ctx = {
    .region        = MEDIC_REGION,
    .bad_blk_cntr  = 0,
    .total_blk_nbr = region_page_nbr[MEDIC_REGION] / BLOCK_SIZE,
    .rd_page_addr  = region_min_addr[MEDIC_REGION],
    .wr_page_addr  = region_min_addr[MEDIC_REGION],
};

static uint32_t block_addr_get(const uint32_t page_addr)
{
    return (page_addr / BLOCK_SIZE) * BLOCK_SIZE;
}

static uint32_t next_block_addr_get(const flash_region_t region, const uint32_t page_addr)
{
    uint32_t next_blk_addr = block_addr_get(page_addr) + BLOCK_SIZE;
    return ((next_blk_addr - region_min_addr[region]) % region_page_nbr[region]) + region_min_addr[region];
}

static uint32_t next_page_addr_get(flash_region_t region, uint32_t page_addr)
{
    if (++page_addr > region_max_addr[region])
    {
        page_addr = region_min_addr[region];
    }

    return page_addr;
}

uint32_t flash_data_region_used_pages(void)
{
    // TODO: Critical section!
    uint32_t used_pages = (m_data_region_ctx.wr_page_addr >= m_data_region_ctx.rd_page_addr)
                              ? m_data_region_ctx.wr_page_addr - m_data_region_ctx.rd_page_addr
                              : region_page_nbr[MEDIC_REGION] - m_data_region_ctx.rd_page_addr + m_data_region_ctx.wr_page_addr;

    return used_pages;
}

uint8_t flash_data_region_percent(void)
{
    return (flash_data_region_used_pages() * 100) / region_page_nbr[MEDIC_REGION];
}

ret_code_t flash_data_write(const uint8_t *const buf, uint16_t size)
{
    // TODO: Critical section!
    if (m_data_region_ctx.wr_page_addr % BLOCK_SIZE == 0)
    {
        uint32_t wr_blk_addr;

        while (block_is_invalid(wr_blk_addr = m_data_region_ctx.wr_page_addr))
        {
            if (++m_data_region_ctx.bad_blk_cntr >= m_data_region_ctx.total_blk_nbr)
            {
                INFO("[%s] No valid block in flash region %u (%u bad blocks)!", __func__, m_data_region_ctx.region,
                     m_data_region_ctx.total_blk_nbr);
                m_data_region_ctx.bad_blk_cntr = m_data_region_ctx.total_blk_nbr;
                return NRF_ERROR_INTERNAL;
            }

            INFO("[%s] Flash invalid block at address %u (%u bad blocks)!", __func__, wr_blk_addr, m_data_region_ctx.bad_blk_cntr);

            m_data_region_ctx.wr_page_addr = next_block_addr_get(m_data_region_ctx.region, (const uint32_t)wr_blk_addr);

            if (m_data_region_ctx.rd_page_addr == wr_blk_addr)
            {
                m_data_region_ctx.rd_page_addr = m_data_region_ctx.wr_page_addr;
            }
            else if (m_data_region_ctx.wr_page_addr == block_addr_get(m_data_region_ctx.rd_page_addr))
            {
                m_data_region_ctx.rd_page_addr =
                    next_block_addr_get(m_data_region_ctx.region, (const uint32_t)m_data_region_ctx.rd_page_addr);
            }
        }

        if (flash_erase_block(m_data_region_ctx.wr_page_addr) != NRF_SUCCESS)
        {
            INFO("[%s] Flash block erase error (block addr: %u)", __func__, m_data_region_ctx.wr_page_addr);
            return NRF_ERROR_INTERNAL;
        }
    }

    INFO("[%s]  Write data to flash addr (rd:%u/wr:%u)", __func__, m_data_region_ctx.rd_page_addr, m_data_region_ctx.wr_page_addr);

    ret_code_t err_code = flash_page_write(m_data_region_ctx.wr_page_addr, (uint8_t *)buf, size);
    if (err_code != NRF_SUCCESS)
    {
        INFO("[%s] Flash page write (addr:%u) error (%d) !", __func__, m_data_region_ctx.wr_page_addr, err_code);
    }

    m_data_region_ctx.wr_page_addr = next_page_addr_get(m_data_region_ctx.region, m_data_region_ctx.wr_page_addr);

    if (m_data_region_ctx.wr_page_addr == m_data_region_ctx.rd_page_addr)
    {
        m_data_region_ctx.rd_page_addr = next_page_addr_get(m_data_region_ctx.region, m_data_region_ctx.rd_page_addr);
    }

    if ((m_data_region_ctx.wr_page_addr % BLOCK_SIZE == 0) &&
        (m_data_region_ctx.wr_page_addr == block_addr_get(m_data_region_ctx.rd_page_addr)))
    {
        m_data_region_ctx.rd_page_addr = next_block_addr_get(m_data_region_ctx.region, (const uint32_t)m_data_region_ctx.rd_page_addr);
    }

    return err_code;
}

ret_code_t flash_data_read(uint8_t *const buf, uint16_t size, uint32_t *page_addr)
{
    if (page_addr == NULL)
    {
        return NRF_ERROR_NULL;
    }

    // TODO: Critical section!
    if (m_data_region_ctx.rd_page_addr == m_data_region_ctx.wr_page_addr)
    {
        return NRF_ERROR_NO_MEM;
    }

    if (m_data_region_ctx.rd_page_addr % BLOCK_SIZE == 0)
    {
        uint32_t rd_blk_addr;

        while (block_is_invalid(rd_blk_addr = m_data_region_ctx.rd_page_addr))
        {
            INFO("[%s] Flash invalid block at address %u !\n", __func__, rd_blk_addr);

            m_data_region_ctx.rd_page_addr = next_block_addr_get(m_data_region_ctx.region, (const uint32_t)rd_blk_addr);

            if (block_addr_get(m_data_region_ctx.wr_page_addr) == rd_blk_addr)
            {
                m_data_region_ctx.wr_page_addr = m_data_region_ctx.rd_page_addr;
            }

            if (m_data_region_ctx.rd_page_addr == m_data_region_ctx.wr_page_addr)
            {
                return NRF_ERROR_NO_MEM;
            }
        }
    }

    INFO("[%s] Read data from flash addr (r:%d/w:%d)\n", __func__, m_data_region_ctx.rd_page_addr, m_data_region_ctx.wr_page_addr);

    *page_addr                     = m_data_region_ctx.rd_page_addr;
    ret_code_t err_code            = flash_page_read(m_data_region_ctx.rd_page_addr, buf, size);
    m_data_region_ctx.rd_page_addr = next_page_addr_get(m_data_region_ctx.region, m_data_region_ctx.rd_page_addr);
    if (err_code != NRF_SUCCESS)
    {
        INFO("[%s] Flash page read (addr:%u) error (%d) !\n", __func__, m_data_region_ctx.rd_page_addr, err_code);
        return NRF_ERROR_INTERNAL;
    }

    return NRF_SUCCESS;
}

void flash_data_region_ctx_get(flash_region_ctx_t *const data_region_ctx)
{
    if (data_region_ctx != NULL)
    {
        CRITICAL_REGION_ENTER();
        {
            memcpy((uint8_t *)data_region_ctx, (uint8_t *)&m_data_region_ctx, sizeof(flash_region_ctx_t));
        }
        CRITICAL_REGION_EXIT();
    }
}

void flash_data_region_ctx_set(flash_region_ctx_t data_region_ctx)
{
    if ((data_region_ctx.region == MEDIC_REGION) && (data_region_ctx.bad_blk_cntr <= data_region_ctx.total_blk_nbr) &&
        (data_region_ctx.total_blk_nbr == (region_page_nbr[MEDIC_REGION] / BLOCK_SIZE)) &&
        (data_region_ctx.rd_page_addr >= region_min_addr[MEDIC_REGION] && data_region_ctx.rd_page_addr <= region_max_addr[MEDIC_REGION]) &&
        (data_region_ctx.wr_page_addr >= region_min_addr[MEDIC_REGION] && data_region_ctx.wr_page_addr <= region_max_addr[MEDIC_REGION]))
    {
        CRITICAL_REGION_ENTER();
        {
            memcpy((uint8_t *)&m_data_region_ctx, (uint8_t *)&data_region_ctx, sizeof(flash_region_ctx_t));
        }
        CRITICAL_REGION_EXIT();
    }
}

void flash_data_region_ctx_reset(void)
{
    CRITICAL_REGION_ENTER();
    {
        m_data_region_ctx.region        = MEDIC_REGION;
        m_data_region_ctx.bad_blk_cntr  = 0;
        m_data_region_ctx.total_blk_nbr = region_page_nbr[MEDIC_REGION] / BLOCK_SIZE;
        m_data_region_ctx.rd_page_addr  = region_min_addr[MEDIC_REGION];
        m_data_region_ctx.wr_page_addr  = region_min_addr[MEDIC_REGION];
    }
    CRITICAL_REGION_EXIT();
}