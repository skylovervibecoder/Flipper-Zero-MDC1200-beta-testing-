#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Costanti MDC1200
#define MDC1200_BAUD_RATE       1200
#define MDC1200_MARK_FREQ       1200
#define MDC1200_SPACE_FREQ      1800
#define MDC1200_SYNC_WORD       0x07090A0B0CULL
#define MDC1200_FRAME_BITS      112
#define MDC1200_FRAME_BYTES     14

// Pi greco come float per evitare double-promotion
#define MDC1200_PI              3.14159265358979323846f

typedef enum {
    Mdc1200OpcodeEmergency      = 0x00,
    Mdc1200OpcodePttId          = 0x01,
    Mdc1200OpcodeCallAlert      = 0x02,
    Mdc1200OpcodeRadioCheck     = 0x03,
    Mdc1200OpcodeSelectiveCall  = 0x04,
    Mdc1200OpcodeRemoteMonitor  = 0x05,
    Mdc1200OpcodeRadioDisable   = 0x2B,
    Mdc1200OpcodeRadioEnable    = 0x2B,
} Mdc1200Opcode;

#define MDC1200_ARG_STUN   0x00
#define MDC1200_ARG_UNSTUN 0x0C

typedef struct {
    uint8_t  op;
    uint8_t  arg;
    uint16_t unit_id;
    bool     crc_ok;
} Mdc1200Decoded;

uint16_t mdc1200_crc16(const uint8_t* data, size_t len);
void     mdc1200_build_frame(uint8_t* frame, uint8_t op, uint8_t arg, uint16_t unit_id);
void     mdc1200_frame_to_bits(const uint8_t* frame, uint8_t* bits, size_t* bit_count);
void     mdc1200_bits_to_ffsk(
    const uint8_t* bits,
    size_t bit_count,
    int16_t* samples,
    size_t* sample_count,
    uint32_t sample_rate);
bool     mdc1200_decode_frame(const uint8_t* bits, size_t bit_count, Mdc1200Decoded* out);
const char* mdc1200_opcode_name(uint8_t op, uint8_t arg);