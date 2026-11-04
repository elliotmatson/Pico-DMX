/*
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "DmxFrameParser.h"

#include <string.h>

namespace dmxstream {

bool DmxFrameParser::finish() {
    if (_count > kMaxSlots) {
        _count = kMaxSlots;  // overlong packet: keep the first 513 slots
    }
    const bool deliver = _inPacket && _count > 0 && !_bad;
    if (_inPacket && _count > 0 && _bad) {
        _stats.dropped++;
    }
    if (deliver) {
        memcpy(_out, _buf, _count);
        _outCount = _count;
        _stats.packets++;
    }
    _count = 0;
    _bad = false;
    return deliver;
}

bool DmxFrameParser::feed(uint32_t word) {
    const uint8_t byte = word & 0xFF;
    if (word & kRxBreak) {
        // Break: closes the packet in progress and opens a new one.
        const bool done = finish();
        _inPacket = true;
        _stats.breaks++;
        return done;
    }
    if (word & kRxFramingError) {
        _stats.framingErrors++;
        _bad = true;
        return false;
    }
    if (!_inPacket) {
        return false;  // no break seen yet: we joined mid-packet
    }
    if (_count < kMaxSlots) {
        _buf[_count++] = byte;
    } else if (_count == kMaxSlots) {
        _stats.overlong++;
        _count++;  // count once; further slots are ignored
    }
    return false;
}

bool DmxFrameParser::flush() {
    const bool done = finish();
    _inPacket = false;  // wait for a fresh break
    return done;
}

}  // namespace dmxstream
