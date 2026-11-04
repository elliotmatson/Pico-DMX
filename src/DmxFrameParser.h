/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * DmxFrameParser: turns the word stream from the DmxReceive PIO program into
 * DMX packets. Pure C++ with no hardware dependencies, so it tests on a host.
 *
 * A packet starts at a break and ends at the next break (or when the line
 * goes quiet and the caller flushes). Packets containing a framing error other
 * than the break itself are counted and dropped.
 */

#ifndef DMX_FRAME_PARSER_H
#define DMX_FRAME_PARSER_H

#include <stddef.h>
#include <stdint.h>

namespace dmxstream {

constexpr uint32_t kRxFramingError = 1u << 31;
constexpr uint32_t kRxBreak = 1u << 30;

class DmxFrameParser {
public:
    static constexpr uint16_t kMaxSlots = 513;  // start code + 512

    struct Stats {
        uint32_t packets;        // complete packets delivered
        uint32_t breaks;
        uint32_t framingErrors;  // bad stop bit on a non-zero byte
        uint32_t dropped;        // packets discarded because of errors
        uint32_t overlong;       // packets with more than 513 slots (truncated)
    };

    // Feed one word. Returns true when a packet completed; it is then in
    // slots()/slotCount() until the next feed() or flush().
    bool feed(uint32_t word);

    // The line went quiet: deliver the packet in progress, if any.
    bool flush();

    const uint8_t* slots() const { return _out; }
    uint16_t slotCount() const { return _outCount; }
    Stats stats() const { return _stats; }

private:
    bool finish();

    uint8_t _buf[kMaxSlots] = {};
    uint8_t _out[kMaxSlots] = {};
    uint16_t _count = 0;
    uint16_t _outCount = 0;
    bool _inPacket = false;
    bool _bad = false;
    Stats _stats{};
};

}  // namespace dmxstream

#endif  // DMX_FRAME_PARSER_H
