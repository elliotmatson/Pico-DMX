/*
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "DmxFrameEncoder.h"

// The encoder runs once per frame on the output core. Build it for speed and,
// on the Pico, run it from RAM so flash (XIP) cache misses don't stall it.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC optimize("O2")
#endif
#if __has_include("pico.h")
#include "pico.h"
#define DMX_ENCODER_RAM(fn) __not_in_flash_func(fn)
#else
#define DMX_ENCODER_RAM(fn) fn
#endif

namespace dmxstream {

namespace {

uint32_t usToWords(uint32_t us) { return (us + kBitUs - 1) / kBitUs; }

uint32_t breakWords(const Lane& l) { return usToWords(l.breakUs < kMinBreakUs ? kMinBreakUs : l.breakUs); }
uint32_t mabWords(const Lane& l) { return usToWords(l.mabUs < kMinMabUs ? kMinMabUs : l.mabUs); }
uint16_t slotCount(const Lane& l) { return l.slotCount > kMaxSlots ? kMaxSlots : l.slotCount; }

}  // namespace

uint32_t laneWords(const Lane& l) {
    return breakWords(l) + mabWords(l) + slotCount(l) * kWordsPerSlot;
}

// Idle pattern (data at mark, DE high on enabled lanes) and frame length.
static size_t frameSetup(const Lane* lanes, size_t count, uint32_t& idle) {
    idle = 0;
    size_t total = 0;
    for (size_t i = 0; i < count; i++) {
        const Lane& l = lanes[i];
        if (l.dataBit >= 0) {
            idle |= 1u << l.dataBit;
        }
        if (l.enabled && l.deBit >= 0) {
            idle |= 1u << l.deBit;
        }
        if (l.enabled) {
            const uint32_t w = laneWords(l);
            if (w > total) {
                total = w;
            }
        }
    }
    return total == 0 ? 1 : total;  // one idle word keeps the stream well-defined
}

namespace detail {

size_t DMX_ENCODER_RAM(encodeFramePerLane)(uint32_t* out, size_t capacity, const Lane* lanes, size_t count) {
    uint32_t idle;
    const size_t total = frameSetup(lanes, count, idle);
    if (total > capacity) {
        return 0;
    }
    for (size_t w = 0; w < total; w++) {
        out[w] = idle;
    }
    // Each lane only clears bits: break, start bits and zero data bits.
    for (size_t i = 0; i < count; i++) {
        const Lane& l = lanes[i];
        if (!l.enabled || l.dataBit < 0) {
            continue;
        }
        const uint32_t clear = ~(1u << l.dataBit);
        const uint32_t brk = breakWords(l);
        for (uint32_t w = 0; w < brk; w++) {
            out[w] &= clear;
        }
        uint32_t* p = out + brk + mabWords(l);
        const uint16_t n = slotCount(l);
        for (uint16_t s = 0; s < n; s++, p += kWordsPerSlot) {
            p[0] &= clear;  // start bit
            const uint8_t v = l.slots ? l.slots[s] : 0;
            for (uint32_t b = 0; b < 8; b++) {
                if (!(v & (1u << b))) {
                    p[1 + b] &= clear;  // LSB first
                }
            }
            // p[9], p[10]: stop bits stay at mark
        }
    }
    return total;
}

}  // namespace detail

namespace {

// Transpose an 8x8 bit matrix held as 8 bytes (byte r = row r, bit c =
// column c): afterwards byte c holds column c, with row r at bit r.
inline uint64_t transpose8(uint64_t x) {
    uint64_t t;
    t = (x ^ (x >> 7)) & 0x00AA00AA00AA00AAull;
    x ^= t ^ (t << 7);
    t = (x ^ (x >> 14)) & 0x0000CCCC0000CCCCull;
    x ^= t ^ (t << 14);
    t = (x ^ (x >> 28)) & 0x00000000F0F0F0F0ull;
    x ^= t ^ (t << 28);
    return x;
}

}  // namespace

size_t DMX_ENCODER_RAM(encodeFrame)(uint32_t* out, size_t capacity, const Lane* lanes, size_t count) {
    // Fast path only when every enabled lane starts its slots at the same word.
    int brk = -1, mab = -1;
    for (size_t i = 0; i < count; i++) {
        const Lane& l = lanes[i];
        if (!l.enabled || l.dataBit < 0) {
            continue;
        }
        const int b = (int)breakWords(l), m = (int)mabWords(l);
        if (brk < 0) {
            brk = b, mab = m;
        } else if (b != brk || m != mab) {
            return detail::encodeFramePerLane(out, capacity, lanes, count);
        }
    }

    uint32_t idle;
    const size_t total = frameSetup(lanes, count, idle);
    if (total > capacity) {
        return 0;
    }
    if (brk < 0) {
        out[0] = idle;
        return 1;
    }

    // Compact list of active lanes; sorted order is not required.
    struct Active {
        const uint8_t* slots;
        uint16_t n;
        uint8_t group;  // dataBit / 8
        uint8_t shift;  // dataBit % 8
        uint32_t bit;
    } act[32];
    size_t na = 0;
    uint32_t enabledData = 0;
    uint32_t groupsUsed = 0;
    uint16_t maxSlots = 0;
    for (size_t i = 0; i < count && na < 32; i++) {
        const Lane& l = lanes[i];
        if (!l.enabled || l.dataBit < 0) {
            continue;
        }
        act[na].slots = l.slots;
        act[na].n = slotCount(l);
        act[na].group = (uint8_t)(l.dataBit >> 3);
        act[na].shift = (uint8_t)(l.dataBit & 7);
        act[na].bit = 1u << l.dataBit;
        enabledData |= act[na].bit;
        groupsUsed |= 1u << act[na].group;
        if (act[na].n > maxSlots) {
            maxSlots = act[na].n;
        }
        na++;
    }

    uint32_t* p = out;
    const uint32_t brkWord = idle & ~enabledData;
    for (int w = 0; w < brk; w++) {
        *p++ = brkWord;
    }
    for (int w = 0; w < mab; w++) {
        *p++ = idle;
    }
    for (uint16_t s = 0; s < maxSlots; s++, p += kWordsPerSlot) {
        // Row r of group g = inverted slot byte of the lane on word bit 8g+r,
        // so after transposing, byte b of group g = that group's zero bits
        // for data bit b.
        uint64_t rows[4] = {0, 0, 0, 0};
        uint32_t live = 0;  // lanes still sending at this slot
        for (size_t a = 0; a < na; a++) {
            if (s >= act[a].n) {
                continue;
            }
            live |= act[a].bit;
            const uint8_t v = act[a].slots ? act[a].slots[s] : 0;
            rows[act[a].group] |= (uint64_t)(uint8_t)~v << (8 * act[a].shift);
        }
        uint32_t zeros[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        for (uint32_t g = 0; g < 4; g++) {
            if (!(groupsUsed & (1u << g))) {
                continue;
            }
            const uint64_t t = transpose8(rows[g]);
            const uint32_t lo = (uint32_t)t, hi = (uint32_t)(t >> 32);
            const uint32_t sh = 8 * g;
            zeros[0] |= (lo & 0xFF) << sh;
            zeros[1] |= ((lo >> 8) & 0xFF) << sh;
            zeros[2] |= ((lo >> 16) & 0xFF) << sh;
            zeros[3] |= (lo >> 24) << sh;
            zeros[4] |= (hi & 0xFF) << sh;
            zeros[5] |= ((hi >> 8) & 0xFF) << sh;
            zeros[6] |= ((hi >> 16) & 0xFF) << sh;
            zeros[7] |= (hi >> 24) << sh;
        }
        p[0] = idle & ~live;  // start bit
        for (uint32_t bit = 0; bit < 8; bit++) {
            p[1 + bit] = idle & ~(zeros[bit] & live);
        }
        p[9] = idle;
        p[10] = idle;
    }
    // Lanes may end at different slots; the remainder of `total` is idle.
    for (uint32_t* e = out + total; p < e; p++) {
        *p = idle;
    }
    return total;
}

}  // namespace dmxstream
