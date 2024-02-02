#ifndef FLASH_DRV_H
#define FLASH_DRV_H

#include <stdint.h>

#include "spi_flash.h"

typedef enum flash_region_tag
{
    AUTO_DIAG_REGION = 0,
    TEMP_REGION,
    MEM_DUMP_REGION,
    DATA_SESSION_REGION,
    MEDIC_REGION,
    FLASH_REGION_NBR
} flash_region_t;

typedef struct flash_region_ctx_tag
{
    flash_region_t    region;
    uint16_t          total_blk_nbr;
    volatile uint16_t bad_blk_cntr;
    volatile uint32_t rd_page_addr;
    volatile uint32_t wr_page_addr;
} flash_region_ctx_t;

static const uint32_t region_page_nbr[FLASH_REGION_NBR] = {
    [AUTO_DIAG_REGION] = AUTO_DIAG_PAGE_NBR,       [TEMP_REGION] = TEMP_PAGE_NBR,   [MEM_DUMP_REGION] = MEM_DUMP_PAGE_NBR,
    [DATA_SESSION_REGION] = DATA_SESSION_PAGE_NBR, [MEDIC_REGION] = MEDIC_PAGE_NBR,
};

static const uint32_t region_min_addr[FLASH_REGION_NBR] = {
    [AUTO_DIAG_REGION] = MIN_AUTO_DIAG_ADDR,       [TEMP_REGION] = MIN_TEMP_ADDR,   [MEM_DUMP_REGION] = MIN_MEM_DUMP_ADDR,
    [DATA_SESSION_REGION] = MIN_DATA_SESSION_ADDR, [MEDIC_REGION] = MIN_MEDIC_ADDR,
};

static const uint32_t region_max_addr[FLASH_REGION_NBR] = {
    [AUTO_DIAG_REGION] = MAX_AUTO_DIAG_ADDR,       [TEMP_REGION] = MAX_TEMP_ADDR,   [MEM_DUMP_REGION] = MAX_MEM_DUMP_ADDR,
    [DATA_SESSION_REGION] = MAX_DATA_SESSION_ADDR, [MEDIC_REGION] = MAX_MEDIC_ADDR,
};

ret_code_t flash_data_read(uint8_t *const buf, uint16_t size, uint32_t *page_addr);
ret_code_t flash_data_write(const uint8_t *const buf, uint16_t size);
uint32_t   flash_data_region_used_pages(void);
uint8_t    flash_data_region_percent(void);
void       flash_data_region_ctx_get(flash_region_ctx_t *const data_region_ctx);
void       flash_data_region_ctx_set(flash_region_ctx_t data_region_ctx);
void       flash_data_region_ctx_reset(void);

#endif // FLASH_DRV_H