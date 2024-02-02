#ifndef SPI_FLASH_H
#define SPI_FLASH_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_error.h"
#include "app_util_platform.h"
#include "boards.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_spi.h"
#include "nrf_gpio.h"

#include "debug.h"

#define OP_READ_ID             0x9F
#define OP_RESET               0xFF
#define OP_GET_FEATURE         0x0F
#define OP_SET_FEATURE         0x1F
#define OP_BLOCK_LOCK          0xA0
#define OP_BLOCK_ERASE         0xD8
#define OP_STATUS_REG          0xC0
#define OP_WRITE_EN            0x06
#define OP_WRITE_DI            0x04
#define OP_PRGM_LOAD           0x02
#define OP_PRGM_EXEC           0x10
#define OP_PAGE_READ           0x13
#define OP_CACHE_READ          0x0B

#define FLASH_MAX_DATA_SIZE    2048U

#define ECC_DATA_SIZE          64U
#define CMD_DATA_SIZE          4U
#define SPI_TX_RX_SIZE         (FLASH_MAX_DATA_SIZE + ECC_DATA_SIZE + CMD_DATA_SIZE)

#define BLOCK_SIZE             64U

#define FLASH_TOTAL_BLOCK      1024U
#define AUTO_DIAG_BLOCK_NBR    1U
#define TEMP_BLOCK_NBR         1U
#define MEM_DUMP_BLOCK_NBR     1U
#define DATA_SESSION_BLOCK_NBR 1U
#define MEDIC_BLOCK_NBR        (FLASH_TOTAL_BLOCK - (AUTO_DIAG_BLOCK_NBR + TEMP_BLOCK_NBR + MEM_DUMP_BLOCK_NBR + DATA_SESSION_BLOCK_NBR))

#define FLASH_MIN_ADDR         0x0000U
#define FLASH_MAX_ADDR         0xFFFFU

// AUTO-DIAG DATA ADDR:  0x0000 -> 0x003F
#define MIN_AUTO_DIAG_ADDR     FLASH_MIN_ADDR
#define MAX_AUTO_DIAG_ADDR     (MIN_AUTO_DIAG_ADDR + (AUTO_DIAG_BLOCK_NBR * BLOCK_SIZE) - 1)

// TEMP DATA ADDR:  0x0040 -> 0x007F
#define MIN_TEMP_ADDR          (MAX_AUTO_DIAG_ADDR + 1)
#define MAX_TEMP_ADDR          (MIN_TEMP_ADDR + (TEMP_BLOCK_NBR * BLOCK_SIZE) - 1)

// MEM-DUMP DATA ADDR: 0x0080 -> 0x00BF
#define MIN_MEM_DUMP_ADDR      (MAX_TEMP_ADDR + 1)
#define MAX_MEM_DUMP_ADDR      (MIN_MEM_DUMP_ADDR + (MEM_DUMP_BLOCK_NBR * BLOCK_SIZE) - 1)

// DATA-SESSION DATA ADDR: 0x00C0 -> 0x00FF
#define MIN_DATA_SESSION_ADDR  (MAX_MEM_DUMP_ADDR + 1)
#define MAX_DATA_SESSION_ADDR  (MIN_DATA_SESSION_ADDR + (DATA_SESSION_BLOCK_NBR * BLOCK_SIZE) - 1)

// MEDIC DATA ADDR: 0x0100 -> 0xFFFF
#define MIN_MEDIC_ADDR         (MAX_DATA_SESSION_ADDR + 1)
#define MAX_MEDIC_ADDR         FLASH_MAX_ADDR

#define PAGE_NBR               (FLASH_MAX_ADDR - FLASH_MIN_ADDR + 1)
#define AUTO_DIAG_PAGE_NBR     (MAX_AUTO_DIAG_ADDR - MIN_AUTO_DIAG_ADDR + 1)
#define TEMP_PAGE_NBR          (MAX_TEMP_ADDR - MIN_TEMP_ADDR + 1)
#define MEM_DUMP_PAGE_NBR      (MAX_MEM_DUMP_ADDR - MIN_MEM_DUMP_ADDR + 1)
#define DATA_SESSION_PAGE_NBR  (MAX_DATA_SESSION_ADDR - MIN_DATA_SESSION_ADDR + 1)
#define MEDIC_PAGE_NBR         (MAX_MEDIC_ADDR - MIN_MEDIC_ADDR + 1)

#define PAYLOAD_SIZE           255U

#define FLASH_TIMEOUT          0x55730U // near 50 ms
#define FLASH_BLOCKS_TIMEOUT   20U

void flash_spi_init(void);
void flash_spi_uninit(void);
void flash_init(void);
void flash_uninit(void);

int32_t flash_page_write(const uint32_t addr, uint8_t *in_buff, uint16_t size);
int32_t flash_page_read(const uint32_t addr, uint8_t *out_buff, uint16_t size);
int32_t flash_erase_block(const uint32_t addr);
bool    block_is_invalid(const uint32_t addr);
uint8_t flash_read_manufacturer_id(void);
uint8_t flash_read_device_id(void);

#endif // SPI_FLASH_H
