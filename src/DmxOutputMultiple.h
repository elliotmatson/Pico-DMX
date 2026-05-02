/*
 * Copyright (c) 2021 Jostein Løwer
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * DmxOutputMultiple<N> — drive N DMX universes in parallel from a single PIO state machine.
 *
 * Each universe is mapped to one GPIO pin; N consecutive pins starting at the base pin
 * supplied to begin(). The PIO program uses MOV and OUT instructions (no side-set), allowing
 * up to 32 simultaneous universes from a single state machine.
 *
 * Supported values of N: 2, 4, 8, 16, 32.
 *
 * IMPORTANT – Generated headers required
 * ---------------------------------------
 * This header includes five auto-generated PIO headers that are produced by running pioasm
 * on the .pio source files in the extras/ directory:
 *
 *   pioasm extras/DmxOutput2.pio  src/DmxOutput2.pio.h
 *   pioasm extras/DmxOutput4.pio  src/DmxOutput4.pio.h
 *   pioasm extras/DmxOutput8.pio  src/DmxOutput8.pio.h
 *   pioasm extras/DmxOutput16.pio src/DmxOutput16.pio.h
 *   pioasm extras/DmxOutput32.pio src/DmxOutput32.pio.h
 *
 * Data format passed to write()
 * ------------------------------
 * universes[k] is a byte array of length 'length' for universe k (k = 0 .. N-1).
 * universes[k][0]    = DMX start code (0x00 for standard DMX data).
 * universes[k][1..n] = channel values.
 *
 * Internally the library packs these arrays into a bit-plane frame buffer before
 * the DMA transfer starts.  One 32-bit word is produced per bit-time (4 µs):
 *   bit k of the word drives pin (OUT_BASE + k)  =>  one bit from universe k.
 *
 * Each byte expands to 11 words:
 *   word  0      : start bit  (all pins LOW  = 0x00000000)
 *   words 1..8   : data bits, LSB first
 *   words 9..10  : stop bits  (all pins HIGH = lower N bits set)
 */

#ifndef DMX_OUTPUT_MULTIPLE_H
#define DMX_OUTPUT_MULTIPLE_H

#if defined(ARDUINO_ARCH_MBED)
  #include <dma.h>
  #include <pio.h>
  #include <clocks.h>
#else
  #ifdef ARDUINO
    #include <Arduino.h>
  #endif
  #include "hardware/dma.h"
  #include "hardware/pio.h"
  #include "hardware/clocks.h"
#endif

#include "DmxOutput2.pio.h"
#include "DmxOutput4.pio.h"
#include "DmxOutput8.pio.h"
#include "DmxOutput16.pio.h"
#include "DmxOutput32.pio.h"

#define DMX_UNIVERSE_SIZE 512
#define DMX_SM_FREQ 1000000

// ---------------------------------------------------------------------------
// Traits: maps template parameter N to the correct pioasm-generated symbols.
// Specialisations are provided for N = 2, 4, 8, 16, 32.
// Using any other value of N will produce a compile-time error.
// ---------------------------------------------------------------------------

template<uint N> struct DmxOutputMultiplePio;  // intentionally undefined for unsupported N

template<> struct DmxOutputMultiplePio<2>
{
    static const pio_program_t &program()              { return DmxOutput2_program; }
    static pio_sm_config get_default_config(uint offset) { return DmxOutput2_program_get_default_config(offset); }
};

template<> struct DmxOutputMultiplePio<4>
{
    static const pio_program_t &program()              { return DmxOutput4_program; }
    static pio_sm_config get_default_config(uint offset) { return DmxOutput4_program_get_default_config(offset); }
};

template<> struct DmxOutputMultiplePio<8>
{
    static const pio_program_t &program()              { return DmxOutput8_program; }
    static pio_sm_config get_default_config(uint offset) { return DmxOutput8_program_get_default_config(offset); }
};

template<> struct DmxOutputMultiplePio<16>
{
    static const pio_program_t &program()              { return DmxOutput16_program; }
    static pio_sm_config get_default_config(uint offset) { return DmxOutput16_program_get_default_config(offset); }
};

template<> struct DmxOutputMultiplePio<32>
{
    static const pio_program_t &program()              { return DmxOutput32_program; }
    static pio_sm_config get_default_config(uint offset) { return DmxOutput32_program_get_default_config(offset); }
};

// ---------------------------------------------------------------------------
// DmxOutputMultiple<N>
// ---------------------------------------------------------------------------

template<uint N>
class DmxOutputMultiple
{
    uint     _prgm_offset;
    uint     _pin;
    uint     _sm;
    PIO      _pio;
    uint     _dma;

    // Pre-packed bit-plane frame buffer.
    // Each DMX byte => 11 words (start bit + 8 data bits + 2 stop bits).
    static const uint WORDS_PER_BYTE  = 11;
    static const uint MAX_FRAME_WORDS = (DMX_UNIVERSE_SIZE + 1) * WORDS_PER_BYTE;

    uint32_t _frame[MAX_FRAME_WORDS];

    // Pack N universe byte arrays into the bit-plane frame buffer.
    void _pack_frame(uint8_t **universes, uint length)
    {
        // Lower N bits all set = "all pins HIGH" (stop / MAB word).
        // Use 64-bit shift to avoid overflow when N == 32.
        const uint32_t ones = (uint32_t)((1ull << N) - 1ull);

        uint wi = 0;
        for (uint bi = 0; bi < length; bi++)
        {
            // Start bit: all pins LOW
            _frame[wi++] = 0u;

            // 8 data bits, LSB first
            for (uint bit = 0; bit < 8; bit++)
            {
                uint32_t word = 0;
                for (uint u = 0; u < N; u++)
                {
                    if (universes[u][bi] & (1u << bit))
                        word |= (1u << u);
                }
                _frame[wi++] = word;
            }

            // 2 stop bits: all pins HIGH
            _frame[wi++] = ones;
            _frame[wi++] = ones;
        }
    }

public:
    /*
     * Return codes — only SUCCESS guarantees the instance is ready to use.
     */
    enum return_code
    {
        SUCCESS = 0,

        // No available state machines left in the PIO instance.
        ERR_NO_SM_AVAILABLE = -1,

        // Not enough PIO program memory.
        ERR_INSUFFICIENT_PRGM_MEM = -2,

        // No available DMA channels.
        ERR_NO_DMA_AVAILABLE = -3
    };

    /*
     * Initialise N simultaneous DMX outputs on N consecutive GPIO pins.
     *
     * Param: pin
     *   Base GPIO pin number.  Universe k is mapped to pin (pin + k).
     *   All N pins must be valid GPIO pins on the RPi Pico.
     *
     * Param: pio
     *   PIO instance to use (defaults to pio0).
     *   Each PIO instance has 4 state machines, so one DmxOutputMultiple<N>
     *   instance consumes one state machine from the chosen PIO.
     */
    return_code begin(uint pin, PIO pio = pio0)
    {
        const pio_program_t &prog = DmxOutputMultiplePio<N>::program();

        if (!pio_can_add_program(pio, &prog))
            return ERR_INSUFFICIENT_PRGM_MEM;

        uint prgm_offset = pio_add_program(pio, &prog);

        int sm = pio_claim_unused_sm(pio, false);
        if (sm == -1)
            return ERR_NO_SM_AVAILABLE;

        // Initialise all N output pins: set HIGH (idle/mark state) and configure as outputs.
        for (uint i = 0; i < N; i++)
        {
            pio_gpio_init(pio, pin + i);
            pio_sm_set_pins_with_mask(pio, sm, 1u << (pin + i), 1u << (pin + i));
            pio_sm_set_pindirs_with_mask(pio, sm, 1u << (pin + i), 1u << (pin + i));
        }

        // Build state machine config from pioasm-generated defaults.
        pio_sm_config sm_conf = DmxOutputMultiplePio<N>::get_default_config(prgm_offset);

        // N consecutive OUT pins starting at 'pin'.
        sm_config_set_out_pins(&sm_conf, pin, N);

        // LSB-first shift so that bit k of each FIFO word drives pin (OUT_BASE + k).
        // Autopull enabled with threshold = N so each OUT instruction consumes exactly
        // one FIFO word (the lower N bits).
        sm_config_set_out_shift(&sm_conf, false, true, N);

        // 1 MHz clock (one cycle = 1 µs, one DMX bit = 4 cycles).
        uint clk_div = clock_get_hz(clk_sys) / DMX_SM_FREQ;
        sm_config_set_clkdiv(&sm_conf, clk_div);

        pio_sm_init(pio, sm, prgm_offset, &sm_conf);
        pio_sm_set_enabled(pio, sm, true);

        // Claim a DMA channel.
        int dma = dma_claim_unused_channel(false);
        if (dma == -1)
            return ERR_NO_DMA_AVAILABLE;

        dma_channel_config dma_conf = dma_channel_get_default_config(dma);

        // 32-bit transfers to match the FIFO word width.
        channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_32);

        // Pace transfers to the PIO TX FIFO.
        channel_config_set_dreq(&dma_conf, pio_get_dreq(pio, sm, true));

        dma_channel_set_write_addr(dma, &pio->txf[sm], false);
        dma_channel_set_config(dma, &dma_conf, false);

        _prgm_offset = prgm_offset;
        _pio         = pio;
        _sm          = sm;
        _pin         = pin;
        _dma         = dma;

        return SUCCESS;
    }

    /*
     * Write N DMX universes simultaneously.
     *
     * Returns immediately; use busy() to poll for completion.
     *
     * Param: universes
     *   Array of N pointers.  universes[k] points to the byte array for universe k.
     *   universes[k][0]    = start code (0x00 for standard DMX).
     *   universes[k][1..n] = channel values.
     *
     * Param: length
     *   Number of bytes per universe including the start code.
     *   Maximum 513 (start code + 512 channels).
     */
    void write(uint8_t **universes, uint length)
    {
        // Pack universes into the bit-plane frame buffer first so the DMA
        // has valid data before the state machine reaches the wrap loop.
        _pack_frame(universes, length);

        // Reset the state machine to restart from the break condition.
        pio_sm_set_enabled(_pio, _sm, false);
        pio_sm_restart(_pio, _sm);
        pio_sm_exec(_pio, _sm, pio_encode_jmp(_prgm_offset));
        pio_sm_set_enabled(_pio, _sm, true);

        // Start DMA — the FIFO will fill during the 176 µs break + 16 µs MAB,
        // so data is ready the moment the state machine enters the wrap loop.
        dma_channel_transfer_from_buffer_now(_dma, _frame, length * WORDS_PER_BYTE);
    }

    /*
     * Returns true while the DMX output is still transmitting.
     */
    bool busy()
    {
        if (dma_channel_is_busy(_dma))
            return true;
        return !pio_sm_is_tx_fifo_empty(_pio, _sm);
    }

    /*
     * De-initialise this instance, releasing PIO and DMA resources.
     */
    void end()
    {
        pio_sm_set_enabled(_pio, _sm, false);
        pio_remove_program(_pio, &DmxOutputMultiplePio<N>::program(), _prgm_offset);
        dma_channel_unclaim(_dma);
        pio_sm_unclaim(_pio, _sm);
    }
};

#endif // DMX_OUTPUT_MULTIPLE_H
