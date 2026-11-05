/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * PixelFrameEncoder: builds the 16-bit word stream for PixelStream.pio from
 * per-lane byte streams (already in wire order, e.g. G R B per pixel).
 * Plain C++, testable on a host.
 */

#ifndef PIXEL_FRAME_ENCODER_H
#define PIXEL_FRAME_ENCODER_H

#include <stddef.h>
#include <stdint.h>

namespace pixelstream {

constexpr unsigned kMaxLanes = 16;

struct Lane {
    const uint8_t* bytes = nullptr;  // wire order, MSB of each byte sent first
    uint16_t count = 0;              // bytes (pixels x bytes per pixel)
};

// Words needed for `maxBytes` bytes on the longest lane.
constexpr size_t frameWords(size_t maxBytes) { return 2 + maxBytes * 8; }

// Encode one frame. Lane n drives word bit n. Lanes with count 0 are left out
// of the mask and stay low. Returns words written, or 0 if `capacity` is too
// small or no lane has data.
size_t encodeFrame(uint16_t* out, size_t capacity, const Lane* lanes, size_t count);

}  // namespace pixelstream

#endif  // PIXEL_FRAME_ENCODER_H
