#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

#include "app_error.h"
#include "nrf_error.h"
#include "sdk_errors.h"

#include "ble_diagw.h"

#define HTONS(val) ((uint16_t)((((val) & 0xff00) >> 8) | ((((val) & 0x00ff) << 8))))

#define HTONL(val)                                                                                                                         \
    ((((uint32_t)(val) & 0xff000000) >> 24) | (((uint32_t)(val) & 0x00ff0000) >> 8) | (((uint32_t)(val) & 0x0000ff00) << 8) |              \
     (((uint32_t)(val) & 0x000000ff) << 24))

#define NTOHS(val)                       HTONS(val)
#define NTOHL(val)                       HTONL(val)

#define UNITS_TO_MSEC(UNITS, RESOLUTION) (((UNITS) * (RESOLUTION)) / 1000)

static inline void ret_code_verify(ret_code_t err_code)
{
    if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_INVALID_STATE))
    {
        APP_ERROR_CHECK(err_code);
    }
}

uint32_t    swap_(uint32_t v);
double      sum_double(double *buf, size_t len);
double      avg_double(double u, double v);
float       avg_float(float u, float v);
void        double_to_string(uint8_t *buf, size_t len, double d);
int32_t     uint64_to_string(uint64_t num, uint8_t *buffer, uint8_t len);
double      dround(double val, uint8_t dp);
char const *cccd_attr_name(uint16_t cccd_handle);
char const *data_type_name(notif_data_type_t type);

#endif // UTILS_H
