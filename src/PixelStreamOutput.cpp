/*
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "PixelStreamOutput.h"

#include "PixelStream.pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"

namespace {
constexpr uint32_t kSmHz = 8000000;  // 10 cycles per 1.25 us bit
}

PixelStreamOutput::Result PixelStreamOutput::begin(uint pinBase, uint pinCount, PIO pio) {
    if (pinCount == 0 || pinCount > 16 || pinBase + pinCount > NUM_BANK0_GPIOS) {
        return ERR_BAD_PINS;
    }
#if PICO_PIO_VERSION > 0
    if (pinBase + pinCount > 32 && (pinBase < 16 || pio_set_gpio_base(pio, 16) != PICO_OK)) {
        return ERR_BAD_PINS;
    }
#endif
    if (!pio_can_add_program(pio, &PixelStream_program)) {
        return ERR_INSUFFICIENT_PRGM_MEM;
    }
    const int sm = pio_claim_unused_sm(pio, false);
    if (sm < 0) {
        return ERR_NO_SM_AVAILABLE;
    }
    const int dma = dma_claim_unused_channel(false);
    if (dma < 0) {
        pio_sm_unclaim(pio, sm);
        return ERR_NO_DMA_AVAILABLE;
    }
    const uint offset = pio_add_program(pio, &PixelStream_program);

    const uint64_t mask = ((1ull << pinCount) - 1) << pinBase;
    pio_sm_set_pins_with_mask64(pio, sm, 0, mask);  // low = idle / latch
    pio_sm_set_pindirs_with_mask64(pio, sm, mask, mask);

    pio_sm_config c = PixelStream_program_get_default_config(offset);
    sm_config_set_out_pins(&c, pinBase, pinCount);
    sm_config_set_out_shift(&c, true, true, 16);  // shift right, autopull 16
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys) / kSmHz);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);

    dma_channel_config d = dma_channel_get_default_config(dma);
    channel_config_set_transfer_data_size(&d, DMA_SIZE_16);
    channel_config_set_read_increment(&d, true);
    channel_config_set_write_increment(&d, false);
    channel_config_set_dreq(&d, pio_get_dreq(pio, sm, true));
    dma_channel_configure(dma, &d, &pio->txf[sm], nullptr, 0, false);

    _pio = pio;
    _sm = sm;
    _offset = offset;
    _dma = dma;
    _draining = false;
    return SUCCESS;
}

void PixelStreamOutput::end() {
    if (!_pio) {
        return;
    }
    dma_channel_abort(_dma);
    dma_channel_unclaim(_dma);
    pio_sm_set_enabled(_pio, _sm, false);
    pio_remove_program(_pio, &PixelStream_program, _offset);
    pio_sm_unclaim(_pio, _sm);
    _pio = nullptr;
}

bool PixelStreamOutput::send(const uint16_t* words, size_t count) {
    if (!_pio || count < 3 || busy()) {
        return false;
    }
    _draining = false;
    dma_channel_transfer_from_buffer_now(_dma, words, count);
    return true;
}

bool PixelStreamOutput::busy() {
    if (!_pio) {
        return false;
    }
    const uint32_t stallBit = 1u << (PIO_FDEBUG_TXSTALL_LSB + _sm);
    if (dma_channel_is_busy(_dma) || !pio_sm_is_tx_fifo_empty(_pio, _sm)) {
        _draining = false;
        return true;
    }
    // As in DmxStreamOutput: the last word may still be in the OSR. Clear the
    // sticky stall flag once and wait for it to come back.
    if (!_draining) {
        _pio->fdebug = stallBit;
        _draining = true;
        return true;
    }
    return !(_pio->fdebug & stallBit);
}
