/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * PixelStreamOutput: WS2812/SK6812 on up to 16 consecutive pins from one PIO
 * state machine and one DMA channel (see PixelStream.pio and
 * PixelFrameEncoder).
 *
 * The pins' GPIO function is not changed: select this PIO on the pins that
 * should carry pixels (gpio_set_function(pin, pio_get_funcsel(pio))). That lets
 * the same pins carry DMX from another PIO block on other ports.
 */

#ifndef PIXEL_STREAM_OUTPUT_H
#define PIXEL_STREAM_OUTPUT_H

#include <stddef.h>
#include <stdint.h>

#include "hardware/pio.h"

class PixelStreamOutput {
public:
    enum Result {
        SUCCESS = 0,
        ERR_NO_SM_AVAILABLE = -1,
        ERR_INSUFFICIENT_PRGM_MEM = -2,
        ERR_NO_DMA_AVAILABLE = -3,
        ERR_BAD_PINS = -4,
    };

    Result begin(uint pinBase, uint pinCount, PIO pio);
    void end();
    PIO pio() const { return _pio; }

    // Start a frame; the buffer must stay untouched until busy() is false.
    bool send(const uint16_t* words, size_t count);
    // True until the last bit has left the pins.
    bool busy();

private:
    PIO _pio = nullptr;
    uint _sm = 0;
    uint _offset = 0;
    int _dma = -1;
    bool _draining = false;
};

#endif  // PIXEL_STREAM_OUTPUT_H
