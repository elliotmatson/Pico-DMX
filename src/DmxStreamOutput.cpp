/*
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "DmxStreamOutput.h"

#include "DmxStream.pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"

namespace {
constexpr uint32_t kSmHz = 1000000;  // 1 us per PIO cycle, 4 cycles per DMX bit
}

DmxStreamOutput::Result DmxStreamOutput::begin(uint pinBase, uint pinCount, uint32_t idleMask, PIO pio) {
    if (pinCount == 0 || pinCount > 32 || pinBase + pinCount > NUM_BANK0_GPIOS) {
        return ERR_BAD_PINS;
    }
#if PICO_PIO_VERSION > 0
    // RP2350B: a PIO sees a 32-GPIO window starting at 0 or 16.
    if (pinBase + pinCount > 32) {
        if (pinBase < 16 || pio_set_gpio_base(pio, 16) != PICO_OK) {
            return ERR_BAD_PINS;
        }
    }
#else
    if (pinBase + pinCount > 32) {
        return ERR_BAD_PINS;
    }
#endif

    if (!pio_can_add_program(pio, &DmxStream_program)) {
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
    const uint offset = pio_add_program(pio, &DmxStream_program);

    const uint64_t pinMask = ((1ull << pinCount) - 1) << pinBase;
    const uint64_t pinValues = (uint64_t)idleMask << pinBase;
    for (uint i = 0; i < pinCount; i++) {
        pio_gpio_init(pio, pinBase + i);
    }
    pio_sm_set_pins_with_mask64(pio, sm, pinValues, pinMask);
    pio_sm_set_pindirs_with_mask64(pio, sm, pinMask, pinMask);

    pio_sm_config c = DmxStream_program_get_default_config(offset);
    sm_config_set_out_pins(&c, pinBase, pinCount);
    sm_config_set_out_shift(&c, true, true, 32);  // shift right, autopull at 32
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys) / kSmHz);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);

    dma_channel_config d = dma_channel_get_default_config(dma);
    channel_config_set_transfer_data_size(&d, DMA_SIZE_32);
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

void DmxStreamOutput::end() {
    if (!_pio) {
        return;
    }
    dma_channel_abort(_dma);
    dma_channel_unclaim(_dma);
    pio_sm_set_enabled(_pio, _sm, false);
    pio_remove_program(_pio, &DmxStream_program, _offset);
    pio_sm_unclaim(_pio, _sm);
    _pio = nullptr;
    _dma = -1;
}

bool DmxStreamOutput::send(const uint32_t* words, size_t count) {
    if (!_pio || count == 0 || busy()) {
        return false;
    }
    _draining = false;
    dma_channel_transfer_from_buffer_now(_dma, words, count);
    return true;
}

bool DmxStreamOutput::busy() {
    if (!_pio) {
        return false;
    }
    const uint32_t stallBit = 1u << (PIO_FDEBUG_TXSTALL_LSB + _sm);
    if (dma_channel_is_busy(_dma) || !pio_sm_is_tx_fifo_empty(_pio, _sm)) {
        _draining = false;
        return true;
    }
    // DMA is done and the FIFO is empty, but the OSR may still hold the last
    // word. Clear the sticky stall flag once; the state machine sets it again
    // only after that word's 4 us have elapsed and it blocks on the next pull.
    if (!_draining) {
        _pio->fdebug = stallBit;  // write 1 to clear
        _draining = true;
        return true;
    }
    return !(_pio->fdebug & stallBit);
}
