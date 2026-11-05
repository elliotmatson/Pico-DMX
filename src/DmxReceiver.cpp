/*
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "DmxReceiver.h"

#include "DmxReceive.pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "pico/time.h"

namespace {

constexpr uint32_t kSmHz = 1000000;
constexpr uint kMaxReceivers = NUM_PIOS * NUM_PIO_STATE_MACHINES;

// Ring buffers must be aligned to their size for DMA address wrapping.
alignas(DmxReceiver::kRingWords * 4) uint32_t gRings[kMaxReceivers][DmxReceiver::kRingWords];
bool gRingUsed[kMaxReceivers];

// One copy of the program per PIO block, shared by its receivers.
int gProgramOffset[NUM_PIOS] = {};
uint gProgramUsers[NUM_PIOS] = {};

}  // namespace

DmxReceiver::Result DmxReceiver::begin(uint pin, PIO pio, bool claimPin) {
    end();
    const uint pioIndex = pio_get_index(pio);

    int ring = -1;
    for (uint i = 0; i < kMaxReceivers; i++) {
        if (!gRingUsed[i]) {
            ring = (int)i;
            break;
        }
    }
    if (ring < 0) {
        return ERR_NO_SM_AVAILABLE;
    }
    if (gProgramUsers[pioIndex] == 0) {
        if (!pio_can_add_program(pio, &DmxReceive_program)) {
            return ERR_INSUFFICIENT_PRGM_MEM;
        }
        gProgramOffset[pioIndex] = pio_add_program(pio, &DmxReceive_program);
    }
    const int sm = pio_claim_unused_sm(pio, false);
    if (sm < 0) {
        if (gProgramUsers[pioIndex] == 0) {
            pio_remove_program(pio, &DmxReceive_program, gProgramOffset[pioIndex]);
        }
        return ERR_NO_SM_AVAILABLE;
    }
    const int dma = dma_claim_unused_channel(false);
    if (dma < 0) {
        pio_sm_unclaim(pio, sm);
        if (gProgramUsers[pioIndex] == 0) {
            pio_remove_program(pio, &DmxReceive_program, gProgramOffset[pioIndex]);
        }
        return ERR_NO_DMA_AVAILABLE;
    }
    gProgramUsers[pioIndex]++;
    gRingUsed[ring] = true;
    const uint offset = gProgramOffset[pioIndex];

    if (claimPin) {
        pio_gpio_init(pio, pin);
        gpio_pull_up(pin);  // an unconnected input idles at mark
    }
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, false);

    pio_sm_config c = DmxReceive_program_get_default_config(offset);
    sm_config_set_in_pins(&c, pin);
    sm_config_set_jmp_pin(&c, pin);
    sm_config_set_in_shift(&c, true, false, 32);  // shift right, manual push
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys) / kSmHz);
    pio_sm_init(pio, sm, offset, &c);

    _ring = gRings[ring];
    dma_channel_config d = dma_channel_get_default_config(dma);
    channel_config_set_transfer_data_size(&d, DMA_SIZE_32);
    channel_config_set_read_increment(&d, false);
    channel_config_set_write_increment(&d, true);
    channel_config_set_ring(&d, true, __builtin_ctz(kRingWords * 4));  // wrap writes
    channel_config_set_dreq(&d, pio_get_dreq(pio, sm, false));
#if PICO_RP2040
    const uint32_t count = 0x0FFFFFFF;
#else
    const uint32_t count = dma_encode_endless_transfer_count();
#endif
    dma_channel_configure(dma, &d, _ring, &pio->rxf[sm], count, true);

    _pio = pio;
    _sm = sm;
    _offset = offset;
    _pin = pin;
    _dma = dma;
    _rd = 0;
    _words = 0;
    _lastWordUs = time_us_32();
    pio_sm_set_enabled(pio, sm, true);
    return SUCCESS;
}

void DmxReceiver::end() {
    if (!_pio) {
        return;
    }
    const uint pioIndex = pio_get_index(_pio);
    pio_sm_set_enabled(_pio, _sm, false);
    dma_channel_abort(_dma);
    dma_channel_unclaim(_dma);
    pio_sm_unclaim(_pio, _sm);
    if (--gProgramUsers[pioIndex] == 0) {
        pio_remove_program(_pio, &DmxReceive_program, _offset);
    }
    for (uint i = 0; i < kMaxReceivers; i++) {
        if (gRings[i] == _ring) {
            gRingUsed[i] = false;
        }
    }
    _pio = nullptr;
    _dma = -1;
    _ring = nullptr;
}

int DmxReceiver::poll(dmxstream::DmxFrameParser& parser, PacketFn onPacket, void* ctx, uint32_t idleUs) {
    if (!_pio) {
        return 0;
    }
#if PICO_RP2040
    if (!dma_channel_is_busy(_dma)) {
        dma_channel_set_trans_count(_dma, 0x0FFFFFFF, true);
    }
#endif
    const uint32_t wrAddr = dma_hw->ch[_dma].write_addr;
    const uint32_t wr = ((wrAddr - (uint32_t)(uintptr_t)_ring) / 4) & (kRingWords - 1);
    int packets = 0;
    const uint32_t now = time_us_32();
    if (wr != _rd) {
        while (_rd != wr) {
            if (parser.feed(_ring[_rd])) {
                onPacket(ctx, parser.slots(), parser.slotCount());
                packets++;
            }
            _rd = (_rd + 1) & (kRingWords - 1);
            _words++;
        }
        _lastWordUs = now;
    } else if (now - _lastWordUs > idleUs) {
        if (parser.flush()) {
            onPacket(ctx, parser.slots(), parser.slotCount());
            packets++;
        }
        _lastWordUs = now;
    }
    return packets;
}
