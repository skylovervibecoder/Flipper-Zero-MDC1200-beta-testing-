#include "mdc1200_dsp.h"
#include <math.h>
#include <string.h>

// CORRETTO: usa MDC1200_PI (float) invece di M_PI
static void design_bandpass(
    float f0, float bw, float fs,
    float* b0, float* b1, float* b2,
    float* a1, float* a2)
{
    float w0 = 2.0f * MDC1200_PI * f0 / fs;
    float alpha = sinf(w0) * sinhf(logf(2.0f) / 2.0f * bw / f0 * w0 / sinf(w0));
    float cos_w0 = cosf(w0);
    float a0 = 1.0f + alpha;

    *b0 =  alpha / a0;
    *b1 =  0.0f;
    *b2 = -alpha / a0;
    *a1 = -2.0f * cos_w0 / a0;
    *a2 = (1.0f - alpha) / a0;
}

void mdc1200_dsp_init(Mdc1200Dsp* dsp, uint32_t sample_rate) {
    memset(dsp, 0, sizeof(Mdc1200Dsp));
    design_bandpass(1200.0f, 400.0f, (float)sample_rate,
        &dsp->b0_m, &dsp->b1_m, &dsp->b2_m, &dsp->a1_m, &dsp->a2_m);
    design_bandpass(1800.0f, 400.0f, (float)sample_rate,
        &dsp->b0_s, &dsp->b1_s, &dsp->b2_s, &dsp->a1_s, &dsp->a2_s);
    dsp->samples_per_bit = sample_rate / MDC1200_BAUD_RATE;
}

static float biquad(
    float in,
    float b0, float b1, float b2,
    float a1, float a2,
    float* x1, float* x2, float* y1, float* y2)
{
    float out = b0 * in + b1 * (*x1) + b2 * (*x2) - a1 * (*y1) - a2 * (*y2);
    *x2 = *x1; *x1 = in;
    *y2 = *y1; *y1 = out;
    return out;
}

void mdc1200_dsp_feed(Mdc1200Dsp* dsp, const int16_t* samples, size_t count) {
    for(size_t i = 0; i < count; i++) {
        float s = (float)samples[i];
        float m = biquad(s, dsp->b0_m, dsp->b1_m, dsp->b2_m,
            dsp->a1_m, dsp->a2_m,
            &dsp->x1_m, &dsp->x2_m, &dsp->y1_m, &dsp->y2_m);
        float sp = biquad(s, dsp->b0_s, dsp->b1_s, dsp->b2_s,
            dsp->a1_s, dsp->a2_s,
            &dsp->x1_s, &dsp->x2_s, &dsp->y1_s, &dsp->y2_s);
        float diff = m * m - sp * sp;
        dsp->accumulated += diff;
        dsp->sample_count++;
        if(dsp->sample_count >= dsp->samples_per_bit) {
            uint8_t bit = (dsp->accumulated > 0.0f) ? 1 : 0;
            if(dsp->bit_count < sizeof(dsp->bits)) {
                dsp->bits[dsp->bit_count++] = bit;
            }
            dsp->accumulated = 0.0f;
            dsp->sample_count = 0;
        }
    }
}

bool mdc1200_dsp_get_frame(Mdc1200Dsp* dsp, Mdc1200Decoded* out) {
    if(dsp->bit_count < 40 + MDC1200_FRAME_BITS) return false;
    bool ok = mdc1200_decode_frame(dsp->bits, dsp->bit_count, out);
    size_t keep = 40;
    if(dsp->bit_count > keep) {
        memmove(dsp->bits, dsp->bits + dsp->bit_count - keep, keep);
        dsp->bit_count = keep;
    } else {
        dsp->bit_count = 0;
    }
    return ok;
}