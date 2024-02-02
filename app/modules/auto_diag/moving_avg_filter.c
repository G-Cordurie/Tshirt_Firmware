#include "nrf.h"
#include "arm_math.h"
#include "moving_avg_filter.h"

void moving_avg_filter(float32_t *in_sig, float32_t *out_sig, uint32_t sig_len, uint32_t fltr_pts)
{
    int32_t i;
    int32_t j;

    for (i = (int32_t)floor((float32_t)fltr_pts / 2); i < ((int32_t)sig_len) - ((int32_t)floor((float32_t)(fltr_pts - 1) / 2)); i++)
    {
        out_sig[i] = 0;

        for (j = -((int32_t)floor((float32_t)fltr_pts / 2)); j < (int32_t)ceil((float32_t)fltr_pts / 2); j++)
        {
            out_sig[i] += in_sig[i + j];
        }

        out_sig[i] /= fltr_pts;
    }
}

/**
 * @brief Recursive moving average filter:
 *          out_sig[i] = out_sig[i-1] + in_sig[i+p] - in_sig[i-q]
 *              where:
 *              p = (M - 1) / 2
 *              q = p + 1
 *              M = Number of points in moving-average 
 * @param in_sig Input signal buffer
 * @param out_sig Output signal buffer
 * @param sig_len Input signal length
 * @param fltr_pts Number of points in moving-average (M)
 */

void rec_moving_avg_filter(float32_t *in_sig, float32_t *out_sig, uint32_t sig_len, uint32_t fltr_pts)
{
    uint32_t i;
    float64_t acc = 0;

    for (i = 0; i < fltr_pts; i++)
    {
        acc += in_sig[i];
    }

    out_sig[(uint32_t)floor((float32_t)(fltr_pts - 1) / 2)] = acc / fltr_pts;

    for (i = (uint32_t)ceil((float32_t)fltr_pts / 2); i < sig_len - (uint32_t)floor((float32_t)fltr_pts / 2); i++)
    {
        acc += in_sig[i + (uint32_t)floor((float32_t)fltr_pts / 2)] - in_sig[i - (uint32_t)ceil((float32_t)fltr_pts / 2)];
        out_sig[i] = acc / fltr_pts;
    }
}

float64_t cont_rec_moving_avg_filter(float64_t in_sample, moving_avg_t *const moving_avg)
{
    if (moving_avg->sample_cntr < moving_avg->fltr_pts)
    {
        moving_avg->fltr_buffer[moving_avg->sample_cntr++] = in_sample;
        moving_avg->acc += in_sample;

        if (moving_avg->sample_cntr == moving_avg->fltr_pts)
        {
            return moving_avg->acc /= moving_avg->fltr_pts;
        }
        else
        {
            return (float64_t)0.0;
        }
    }

    moving_avg->acc += in_sample - moving_avg->fltr_buffer[0];
    memmove(&moving_avg->fltr_buffer[0], &moving_avg->fltr_buffer[1], (moving_avg->fltr_pts - 1) * sizeof(moving_avg->fltr_buffer[0]));
    moving_avg->fltr_buffer[moving_avg->fltr_pts - 1] = in_sample;

    return moving_avg->acc;
}
