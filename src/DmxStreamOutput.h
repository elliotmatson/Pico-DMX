/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * DmxStreamOutput: drive up to 32 DMX512 lines from one PIO state machine and
 * one DMA channel. Frames are built with DmxFrameEncoder; this class only
 * streams them.
 *
 * Unlike DmxOutput, busy() stays true until the last bit-time has left the
 * pins (it waits for the state machine to stall on an empty FIFO), so a new
 * frame never truncates the previous one. The state machine is never
 * restarted between frames, because break and MAB are part of the data.
 *
 * Requires pico-sdk 2.x (RP2040 or RP2350). Pins may be above GPIO 31 on the
 * RP2350B; all pins of one instance must sit in the same 32-pin PIO window.
 */

#ifndef DMX_STREAM_OUTPUT_H
#define DMX_STREAM_OUTPUT_H

#include <stddef.h>
#include <stdint.h>

#include "hardware/pio.h"

class DmxStreamOutput {
public:
    enum Result {
        SUCCESS = 0,
        ERR_NO_SM_AVAILABLE = -1,
        ERR_INSUFFICIENT_PRGM_MEM = -2,
        ERR_NO_DMA_AVAILABLE = -3,
        ERR_BAD_PINS = -4,
    };

    // pinBase..pinBase+pinCount-1 become OUT pins; word bit n drives
    // pinBase + n. `idleMask` sets their level until the first frame (bit n
    // high = pin high): data pins should idle high, DE pins low.
    Result begin(uint pinBase, uint pinCount, uint32_t idleMask, PIO pio = pio0);
    void end();

    // Start streaming `count` words. The buffer must stay untouched until
    // busy() returns false. Calls while busy() are ignored and return false.
    bool send(const uint32_t* words, size_t count);

    // True while any part of the last frame is still being clocked out.
    bool busy();

    PIO pio() const { return _pio; }
    uint sm() const { return _sm; }

private:
    PIO _pio = nullptr;
    uint _sm = 0;
    uint _offset = 0;
    int _dma = -1;
    bool _draining = false;
};

#endif  // DMX_STREAM_OUTPUT_H
