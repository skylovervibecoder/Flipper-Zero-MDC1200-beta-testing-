# MDC1200 (Momentum)

![MDC1200) (mdc.png)

A Flipper Zero application (FAP) for **encoding, transmitting, receiving, and decoding MDC1200** signaling on PMR446 and FRS channels.  
Built for the **Momentum** firmware.

> ⚠️ **LEGAL DISCLAIMER**  
> Transmitting on PMR446 and FRS frequencies is **strictly regulated** in most countries.  
> Using a non‑certified device (such as the Flipper Zero) to transmit on these bands is **illegal**.  
> This software is provided **for educational and research purposes only**.  
> The author assumes **no responsibility** for any unauthorized or unlawful use.  
> Always comply with your local radio regulations.

---

## Features

- **Full MDC1200 support** – all common opcodes:
  - PTT ID (`0x01`)
  - Emergency (`0x00`)
  - Call Alert (`0x02`)
  - Radio Check (`0x03`)
  - Selective Call (`0x04`)
  - Remote Monitor (`0x05`)
  - Stun / Unstun (`0x2B` with arg `0x00` / `0x0C`)
- **PMR446** (8 channels) and **FRS** (14 channels) frequency tables.
- **Configurable Unit ID** (0–65535).
- **FFSK modulation** (1200/1800 Hz, 1200 baud) with sync word.
- **CRC‑16/CCITT** error checking.
- **Convolutional FEC** (R=1/2, K=7) encoder.
- **Interleaving / de‑interleaving** (7×16 matrix).
- **Real‑time DSP** for reception: dual biquad band‑pass filters + FSK discriminator.
- **Sub‑GHz TX/RX** via the internal CC1101.
- **Chunked transmission** to work around the 60‑byte FIFO limit.
- **User‑friendly GUI** with submenu and status view.

---

## Requirements

- **Flipper Zero** with [Momentum firmware](https://github.com/Next-Flip/Momentum-Firmware) installed.
- **ufbt** (micro Flipper Build Tool) – install via `pip install ufbt`.
- **Sub‑GHz antenna** suitable for UHF (446 MHz / 462–467 MHz).
- (Optional) A **Flipper Zero connected via USB** for direct upload.

---
