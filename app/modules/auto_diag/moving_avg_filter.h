#ifndef MOVING_AVG_FILTER_H
#define MOVING_AVG_FILTER_H

#include "arm_math.h"

typedef struct moving_avg_tag
{
    uint16_t fltr_pts;
    uint16_t sample_cntr;
    float64_t *fltr_buffer;
    float64_t acc;
} moving_avg_t;

void moving_avg_filter(float32_t *in_sig, float32_t *out_sig, uint32_t sig_len, uint32_t fltr_pts);
void rec_moving_avg_filter(float32_t *in_sig, float32_t *out_sig, uint32_t sig_len, uint32_t fltr_pts);
float64_t cont_rec_moving_avg_filter(float64_t in_sample, moving_avg_t *const moving_avg);

#endif // MOVING_AVG_FILTER_H