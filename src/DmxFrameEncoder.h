/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * DmxFrameEncoder: builds the bit-plane buffer streamed by DmxStreamOutput.
 *
 * One 32-bit word per 4 us bit-time; bit n of a word drives OUT pin
 * (base + n). Each lane (DMX line) gets its own data bit and an optional DE
 * bit, its own slot data and its own break / MAB timing. Lanes run in
 * parallel from word 0, so lines with different timing or lengths simply end
 * at different words; a finished lane idles at mark.
 *
 * Pure C++ with no hardware dependencies, so it builds and tests on a host.
 */

#ifndef DMX_FRAME_ENCODER_H
#define DMX_FRAME_ENCODER_H

#include <stddef.h>
#include <stdint.h>

namespace dmxstream {

constexpr uint32_t kBitUs = 4;        // 250 kbaud
constexpr uint32_t kWordsPerSlot = 11;  // start + 8 data + 2 stop
constexpr uint16_t kMaxSlots = 513;   // start code + 512 channels
constexpr uint16_t kMinBreakUs = 92;  // E1.11 transmitter minimum
constexpr uint16_t kMinMabUs = 12;    // E1.11 transmitter minimum
constexpr int8_t kNoBit = -1;

struct Lane {
    bool enabled = false;
    int8_t dataBit = kNoBit;  // word bit driving this lane's DI pin
    int8_t deBit = kNoBit;    // word bit driving DE, or kNoBit if DE is fixed
    uint16_t breakUs = 176;
    uint16_t mabUs = 16;
    const uint8_t* slots = nullptr;  // [0] = start code
    uint16_t slotCount = 0;          // including the start code, <= kMaxSlots
    // RDM: release DE (driver off) as soon as this lane's packet ends instead
    // of holding it for the whole frame, so a responder can reply.
    bool releaseDe = false;
    // A disabled lane normally drops DE (driver off). With holdDe it keeps
    // DE high and its line at mark instead: an output sitting a frame out
    // (slow refresh, Delta mode, stopped on loss), or a lane whose data pin
    // is driven by something else for now (e.g. a pixel stream) but whose
    // transceiver must stay on.
    bool holdDe = false;
};

// Words a lane needs for one frame (break + MAB + slots), after clamping its
// timing to the E1.11 minimums and rounding up to whole bit-times.
uint32_t laneWords(const Lane& lane);

// Largest frame any lane set can produce with the given timing limits.
constexpr uint32_t maxFrameWords(uint16_t maxBreakUs = 1000, uint16_t maxMabUs = 1000) {
    return (maxBreakUs + kBitUs - 1) / kBitUs + (maxMabUs + kBitUs - 1) / kBitUs +
           kMaxSlots * kWordsPerSlot;
}

// Encode one frame for `count` lanes into `out`. Returns the number of words
// written (the longest lane), or 0 if `capacity` is too small.
//
// Disabled lanes hold their data pin at mark and DE (if any) low, or high
// with holdDe. Enabled lanes hold DE high for the whole frame.
//
// When every enabled lane shares the same break and MAB (the usual case) a
// slot-major fast path writes each word once; otherwise each lane is applied
// in turn. Both produce identical output.
//
// `minWords` pads the frame with idle words (e.g. to leave room for RDM
// replies when every lane is short).
size_t encodeFrame(uint32_t* out, size_t capacity, const Lane* lanes, size_t count, size_t minWords = 0);

namespace detail {
// The general per-lane path, exposed so tests can compare it with the fast path.
size_t encodeFramePerLane(uint32_t* out, size_t capacity, const Lane* lanes, size_t count, size_t minWords = 0);
}  // namespace detail

}  // namespace dmxstream

#endif  // DMX_FRAME_ENCODER_H
