#include <float.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "arm_math.h"

#include "debug.h"
#include "utils.h"

uint32_t swap_(uint32_t v)
{
    return ((v & 0xFF) << 24) | ((v & 0xFF00) << 8) | ((v & 0xFF0000) >> 8) | ((v & 0xFF000000) >> 24); // swap uint32_t
}

bool isequal(double u, double v)
{
    return fabs(u - v) <= DBL_EPSILON * fmax(fmax(1.0, fabs(u)), fabs(v));
}

double sum_double(double *buf, size_t len)
{
    if (!buf || !len)
    {
        return 0.0;
    }

    double s = 0.0;
    double c = 0.0;

    for (size_t i = 0; i < len; i++)
    {
        double y = buf[i] - c;
        double t = s + y;
        c        = (t - s) - y;
        s        = t;
    }

    if (isfinite(s))
    {
        return s;
    }

    INFO("[%s] Floating-point computation error!", (uint32_t) __func__);
    return 0.0;
}

double avg_double(double u, double v)
{
    const double c   = 0x1p970;
    double       avg = 0.0;

    if (c <= fabs(u))
    {
        avg = u / 2.0 + v / 2.0;
    }
    else
    {
        avg = (u + v) / 2.0;
    }

    if (isfinite(avg))
    {
        return avg;
    }

    INFO("[%s] Floating-point computation error!", (uint32_t) __func__);
    return 0.0;
}

float avg_float(float u, float v)
{
    const float c   = 0x1p103F;
    float       avg = 0.0F;

    if (c <= fabsf(u))
    {
        avg = u / 2.0F + v / 2.0F;
    }
    else
    {
        avg = (u + v) / 2.0F;
    }

    if (isfinite(avg))
    {
        return avg;
    }

    INFO("[%s] Floating-point computation error!", (uint32_t) __func__);
    return 0.0F;
}

void double_to_string(uint8_t *buf, size_t len, double d)
{
    if (!buf || !len)
    {
        return;
    }

    memset(buf, 0, len);
    sprintf((char *)buf, "%.15lf", d);
}

int32_t uint64_to_string(uint64_t num, uint8_t *buffer, uint8_t len)
{
    if (buffer && len)
    {
        int str_len = 1 + snprintf(NULL, 0, "%llu", (long long unsigned int)num);
        if (len >= str_len)
        {
            memset(buffer, 0, sizeof(buffer));
            return snprintf((char *)buffer, len, "%llu", (long long unsigned int)num);
        }
    }

    return -1;
}

double dround(double val, uint8_t dp)
{
    int  len = 1 + snprintf(NULL, 0, "%.*f", dp, val);
    char buffer[len];
    memset(buffer, 0, sizeof(buffer));
    (void)snprintf(buffer, len, "%.*f", dp, val);
    return atof(buffer);
}

char const *cccd_attr_name(uint16_t cccd_handle)
{
    static char const *str[] = {[0] = "UNKNOWN", [1] = "ECG",      [2] = "BREATH",          [3] = "TEMP",          [4] = "IMP",
                                [5] = "ACC",     [6] = "MEM_DUMP", [7] = "MEM_DUMP_CTRLPT", [8] = "BATTERY_LEVEL", [9] = "SERVICE_CHANGED"};

    switch (cccd_handle)
    {
    case 0x0024:
        return str[1];

    case 0x0027:
        return str[2];

    case 0x0033:
        return str[6];

    case 0x002A:
        return str[3];

    case 0x002D:
        return str[4];

    case 0x0030:
        return str[5];

    case 0x0036:
        return str[7];

    case 0x0017:
        return str[8];

    case 0x000D:
        return str[9];

    default:
        return str[0];
    }
}

char const *data_type_name(notif_data_type_t type)
{
    static char const *str[] = {[UNKNOWN_TYPE] = "UNKNOWN_TYPE", [ECG_TYPE] = "ECG_TYPE",
                                [BREATH_TYPE] = "BREATH_TYPE",   [TEMP_TYPE] = "TEMP_TYPE",
                                [IMP_TYPE] = "IMP_TYPE",         [MEM_DUMP_TYPE] = "MEM_DUMP_TYPE",
                                [ACC_TYPE] = "ACC_TYPE",         [MEM_DUMP_CTRLPT_TYPE] = "MEM_DUMP_CTRLPT_TYPE"};

    if (type > UNKNOWN_TYPE && type < NOTIF_DATA_TYPE_SIZE)
    {
        return str[type];
    }

    return str[UNKNOWN_TYPE];
}