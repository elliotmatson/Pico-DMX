/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * DmxReceiver: one DMX512 input on one PIO state machine, with DMA writing the
 * receiver's words into a ring buffer. poll() feeds new words to a
 * DmxFrameParser, which splits them into packets at each break, so packets of
 * any length (1-513 slots) are handled and no interrupt is needed.
 *
 * Call poll() at least every ~20 ms (the ring holds 512 slots); the output
 * core's main loop is a good place.
 *
 * Requires pico-sdk 2.x. On the RP2350 the DMA runs endlessly; on the RP2040
 * it is re-armed from poll().
 */

#ifndef DMX_RECEIVER_H
#define DMX_RECEIVER_H

#include <stdint.h>

#include "DmxFrameParser.h"
#include "hardware/pio.h"

class DmxReceiver {
public:
    enum Result {
        SUCCESS = 0,
        ERR_NO_SM_AVAILABLE = -1,
        ERR_INSUFFICIENT_PRGM_MEM = -2,
        ERR_NO_DMA_AVAILABLE = -3,
    };

    static constexpr uint32_t kRingWords = 512;  // must be a power of two

    // Start receiving on `pin`. With `claimPin` false the pin's GPIO function is
    // left alone, so the receiver can listen to a pin driven by something else
    // (e.g. a loopback test of an output).
    Result begin(uint pin, PIO pio, bool claimPin = true);
    void end();
    bool running() const { return _pio != nullptr; }
    uint pin() const { return _pin; }

    using PacketFn = void (*)(void* ctx, const uint8_t* slots, uint16_t count);

    // Feed newly received words to `parser`, calling `onPacket` for every
    // packet that completes. Returns the number of packets delivered.
    // `idleUs` flushes a packet in progress once no word has arrived for that
    // long (the line went quiet without a following break).
    int poll(dmxstream::DmxFrameParser& parser, PacketFn onPacket, void* ctx, uint32_t idleUs = 2000);

    uint32_t wordsReceived() const { return _words; }

private:
    PIO _pio = nullptr;
    uint _sm = 0;
    uint _offset = 0;
    uint _pin = 0;
    int _dma = -1;
    uint32_t* _ring = nullptr;
    uint32_t _rd = 0;  // next ring index to read
    uint32_t _words = 0;
    uint32_t _lastWordUs = 0;
};

#endif  // DMX_RECEIVER_H
