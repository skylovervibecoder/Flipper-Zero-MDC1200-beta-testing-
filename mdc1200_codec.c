#include "mdc1200_codec.h"
#include <string.h>
#include <math.h>

uint16_t mdc1200_crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0x0000;
    for(size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for(int b = 0; b < 8; b++) {
            if(crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc ^ 0xFFFF;
}

#define MDC1200_FEC_POLY1 0x6D
#define MDC1200_FEC_POLY2 0x4F

static uint8_t fec_shift_reg;

static void fec_reset(void) {
    fec_shift_reg = 0;
}

static void fec_encode_bit(uint8_t in_bit, uint8_t* out1, uint8_t* out2) {
    fec_shift_reg = ((fec_shift_reg << 1) | (in_bit & 1)) & 0x7F;
    uint8_t b1 = 0, b2 = 0;
    for(int i = 0; i < 7; i++) {
        uint8_t sr_bit = (fec_shift_reg >> i) & 1;
        if((MDC1200_FEC_POLY1 >> i) & 1) b1 ^= sr_bit;
        if((MDC1200_FEC_POLY2 >> i) & 1) b2 ^= sr_bit;
    }
    *out1 = b1;
    *out2 = b2;
}

static void mdc1200_encode_fec(const uint8_t* in, size_t in_bits, uint8_t* out, size_t* out_bits) {
    fec_reset();
    memset(out, 0, 16);
    size_t oidx = 0;
    for(size_t i = 0; i < in_bits; i++) {
        uint8_t bit = (in[i / 8] >> (7 - (i % 8))) & 1;
        uint8_t o1, o2;
        fec_encode_bit(bit, &o1, &o2);
        out[oidx / 8] |= (o1 << (7 - (oidx % 8)));
        oidx++;
        out[oidx / 8] |= (o2 << (7 - (oidx % 8)));
        oidx++;
    }
    for(int i = 0; i < 6; i++) {
        uint8_t o1, o2;
        fec_encode_bit(0, &o1, &o2);
        out[oidx / 8] |= (o1 << (7 - (oidx % 8)));
        oidx++;
        out[oidx / 8] |= (o2 << (7 - (oidx % 8)));
        oidx++;
    }
    *out_bits = oidx;
}

void mdc1200_build_frame(uint8_t* frame, uint8_t op, uint8_t arg, uint16_t unit_id) {
    memset(frame, 0, MDC1200_FRAME_BYTES);
    frame[0] = op;
    frame[1] = arg;
    frame[2] = (unit_id >> 8) & 0xFF;
    frame[3] = unit_id & 0xFF;
    uint16_t crc = mdc1200_crc16(frame, 4);
    frame[4] = crc & 0xFF;
    frame[5] = (crc >> 8) & 0xFF;
    uint8_t fec_out[16] = {0};
    size_t fec_bits = 0;
    mdc1200_encode_fec(frame, 48, fec_out, &fec_bits);
    memcpy(frame + 6, fec_out, 8);
}

void mdc1200_frame_to_bits(const uint8_t* frame, uint8_t* bits, size_t* bit_count) {
    size_t idx = 0;
    uint64_t sync = MDC1200_SYNC_WORD;
    for(int i = 39; i >= 0; i--) {
        bits[idx++] = (sync >> i) & 1;
    }
    for(size_t i = 0; i < MDC1200_FRAME_BYTES; i++) {
        for(int b = 7; b >= 0; b--) {
            bits[idx++] = (frame[i] >> b) & 1;
        }
    }
    *bit_count = idx;
}

// CORRETTO: usa float e sinf per evitare double-promotion
void mdc1200_bits_to_ffsk(
    const uint8_t* bits,
    size_t bit_count,
    int16_t* samples,
    size_t* sample_count,
    uint32_t sample_rate)
{
    float samples_per_bit = (float)sample_rate / (float)MDC1200_BAUD_RATE;
    size_t total = (size_t)((float)bit_count * samples_per_bit);
    float phase = 0.0f;

    for(size_t i = 0; i < bit_count; i++) {
        float freq = bits[i] ? (float)MDC1200_MARK_FREQ : (float)MDC1200_SPACE_FREQ;
        float omega = 2.0f * MDC1200_PI * freq / (float)sample_rate;
        size_t start = (size_t)((float)i * samples_per_bit);
        size_t end   = (size_t)((float)(i + 1) * samples_per_bit);
        if(end > total) end = total;
        for(size_t s = start; s < end; s++) {
            samples[s] = (int16_t)(sinf(phase) * 32767.0f);
            phase += omega;
        }
    }
    *sample_count = total;
}

bool mdc1200_decode_frame(const uint8_t* bits, size_t bit_count, Mdc1200Decoded* out) {
    uint64_t sync = MDC1200_SYNC_WORD;
    size_t sync_idx = (size_t)-1;
    for(size_t i = 0; i + 40 <= bit_count; i++) {
        uint64_t candidate = 0;
        for(int j = 0; j < 40; j++) {
            candidate = (candidate << 1) | bits[i + j];
        }
        if(candidate == sync) {
            sync_idx = i + 40;
            break;
        }
    }
    if(sync_idx == (size_t)-1 || sync_idx + MDC1200_FRAME_BITS > bit_count) {
        return false;
    }
    const uint8_t* payload = bits + sync_idx;
    uint8_t deinterleaved[MDC1200_FRAME_BITS];
    for(int col = 0; col < 7; col++) {
        for(int row = 0; row < 16; row++) {
            size_t src = (size_t)col * 16 + row;
            size_t dst = (size_t)row * 7 + col;
            if(src < MDC1200_FRAME_BITS && dst < MDC1200_FRAME_BITS) {
                deinterleaved[dst] = payload[src];
            }
        }
    }
    uint8_t bytes[MDC1200_FRAME_BYTES];
    for(int i = 0; i < MDC1200_FRAME_BYTES; i++) {
        uint8_t b = 0;
        for(int bit = 0; bit < 8; bit++) {
            if(deinterleaved[i * 8 + bit]) b |= (1 << bit);
        }
        bytes[i] = b;
    }
    uint16_t rx_crc = bytes[4] | ((uint16_t)bytes[5] << 8);
    uint16_t calc_crc = mdc1200_crc16(bytes, 4);
    out->op      = bytes[0];
    out->arg     = bytes[1];
    out->unit_id = ((uint16_t)bytes[2] << 8) | bytes[3];
    out->crc_ok  = (rx_crc == calc_crc);
    return true;
}

const char* mdc1200_opcode_name(uint8_t op, uint8_t arg) {
    switch(op) {
    case Mdc1200OpcodeEmergency:     return "Emergency";
    case Mdc1200OpcodePttId:         return "PTT ID";
    case Mdc1200OpcodeCallAlert:     return "Call Alert";
    case Mdc1200OpcodeRadioCheck:    return "Radio Check";
    case Mdc1200OpcodeSelectiveCall: return "Sel. Call";
    case Mdc1200OpcodeRemoteMonitor: return "Remote Mon";
    case Mdc1200OpcodeRadioDisable:
        return (arg == MDC1200_ARG_UNSTUN) ? "Unstun" : "Stun";
    default: return "Unknown";
    }
}