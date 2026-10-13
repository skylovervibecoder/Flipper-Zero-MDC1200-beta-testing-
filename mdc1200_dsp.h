#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "mdc1200_codec.h"

typedef struct {
    float b0_m, b1_m, b2_m, a1_m, a2_m;
    float b0_s, b1_s, b2_s, a1_s, a2_s;
    float x1_m, x2_m, y1_m, y2_m;
    float x1_s, x2_s, y1_s, y2_s;

    uint32_t sample_count;
    uint32_t samples_per_bit;
    float    accumulated;

    uint8_t bits[512];
    size_t  bit_count;
} Mdc1200Dsp;

void mdc1200_dsp_init(Mdc1200Dsp* dsp, uint32_t sample_rate);
void mdc1200_dsp_feed(Mdc1200Dsp* dsp, const int16_t* samples, size_t count);
bool mdc1200_dsp_get_frame(Mdc1200Dsp* dsp, Mdc1200Decoded* out);