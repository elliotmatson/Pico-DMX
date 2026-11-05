/*
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "PixelFrameEncoder.h"

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC optimize("O2")
#endif
#if __has_include("pico.h")
#include "pico.h"
#define PIXEL_ENCODER_RAM(fn) __not_in_flash_func(fn)
#else
#define PIXEL_ENCODER_RAM(fn) fn
#endif

namespace pixelstream {

namespace {

// 8x8 bit transpose: byte r = row r (bit c = column c) -> byte c holds column
// c with row r at bit r.
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

size_t PIXEL_ENCODER_RAM(encodeFrame)(uint16_t* out, size_t capacity, const Lane* lanes, size_t count) {
    if (count > kMaxLanes) {
        count = kMaxLanes;
    }
    uint16_t mask = 0;
    size_t maxBytes = 0;
    for (size_t i = 0; i < count; i++) {
        if (lanes[i].count && lanes[i].bytes) {
            mask |= (uint16_t)(1u << i);
            if (lanes[i].count > maxBytes) {
                maxBytes = lanes[i].count;
            }
        }
    }
    const size_t words = frameWords(maxBytes);
    if (!mask || words > capacity) {
        return 0;
    }
    out[0] = mask;
    out[1] = (uint16_t)(maxBytes * 8 - 1);
    uint16_t* p = out + 2;
    for (size_t b = 0; b < maxBytes; b++, p += 8) {
        // Rows: lanes 0-7 and 8-15, one byte each; lanes past their end send 0.
        uint64_t lo = 0, hi = 0;
        for (size_t i = 0; i < count && i < 8; i++) {
            if (b < lanes[i].count) {
                lo |= (uint64_t)lanes[i].bytes[b] << (8 * i);
            }
        }
        for (size_t i = 8; i < count; i++) {
            if (b < lanes[i].count) {
                hi |= (uint64_t)lanes[i].bytes[b] << (8 * (i - 8));
            }
        }
        lo = transpose8(lo);  // byte c = bit c of every lane
        hi = transpose8(hi);
        for (int bit = 7; bit >= 0; bit--) {  // MSB first on the wire
            *p++ = (uint16_t)(((lo >> (8 * bit)) & 0xFF) | (((hi >> (8 * bit)) & 0xFF) << 8));
        }
        p -= 8;
    }
    return words;
}

}  // namespace pixelstream
