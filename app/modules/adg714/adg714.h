#ifndef ADG714_H
#define ADG714_H

#include <stdint.h>

#define ADG714_CMD_LEN 1

void adg714_init(void);
void adg714_spi_init(void);
void adg714_spi_uninit(void);
void adg714_spi_write(uint8_t const *data, uint8_t len);

#endif // ADG714_H
