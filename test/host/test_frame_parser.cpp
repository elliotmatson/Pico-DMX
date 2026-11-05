// Host test for DmxFrameParser, using the encoder to produce realistic input:
// the encoded line is "received" by a model of the DmxReceive PIO program.
//
//   c++ -std=c++17 -I src test/host/test_frame_parser.cpp src/DmxFrameParser.cpp src/DmxFrameEncoder.cpp -o /tmp/p && /tmp/p

#include <cstdio>
#include <random>
#include <vector>

#include "DmxFrameEncoder.h"
#include "DmxFrameParser.h"

using namespace dmxstream;

static int failures = 0;
#define CHECK(cond, ...)                                     \
    do {                                                     \
        if (!(cond)) {                                       \
            std::printf("FAIL %s:%d: ", __FILE__, __LINE__); \
            std::printf(__VA_ARGS__);                        \
            std::printf("\n");                               \
            failures++;                                      \
        }                                                    \
    } while (0)

// Model of DmxReceive.pio at 1 MHz over a line sampled once per microsecond.
static std::vector<uint32_t> receive(const std::vector<int>& line) {
    std::vector<uint32_t> words;
    size_t t = 0;
    auto at = [&](size_t i) { return i < line.size() ? line[i] : 1; };
    while (t < line.size()) {
        while (t < line.size() && at(t) == 1) t++;  // wait 0 pin
        if (t >= line.size()) break;
        const size_t edge = t;
        uint32_t v = 0;
        for (int b = 0; b < 8; b++) v |= (uint32_t)at(edge + 6 + 4 * b) << b;
        if (at(edge + 38)) {  // jmp pin: first stop bit
            words.push_back(v);
            t = edge + 41;
            continue;
        }
        // Watch the line for 11 more 4 us steps, like the low: loop.
        bool early = false;
        size_t s = edge + 40;
        for (int y = 0; y < 11; y++, s += 4) {
            if (at(s)) {
                early = true;
                break;
            }
        }
        if (early) {
            words.push_back(v | kRxFramingError);
            t = s + 1;
        } else {
            words.push_back(kRxBreak);
            t = s;
            while (t < line.size() && at(t) == 0) t++;  // wait 1 pin
        }
    }
    return words;
}

// Expand encoder words (4 us each) for one lane into 1 us samples.
static void appendLane(std::vector<int>& line, const std::vector<uint32_t>& words, int bit) {
    for (uint32_t w : words)
        for (int i = 0; i < 4; i++) line.push_back((w >> bit) & 1);
}

static std::vector<uint32_t> encode(const std::vector<uint8_t>& slots, uint16_t breakUs = 176, uint16_t mabUs = 16) {
    Lane l;
    l.enabled = true;
    l.dataBit = 0;
    l.slots = slots.data();
    l.slotCount = (uint16_t)slots.size();
    l.breakUs = breakUs;
    l.mabUs = mabUs;
    std::vector<uint32_t> buf(maxFrameWords());
    buf.resize(encodeFrame(buf.data(), buf.size(), &l, 1));
    return buf;
}

static void testPacketsOfVaryingLength() {
    std::mt19937 rng(7);
    std::vector<int> line(500, 1);  // idle mark
    std::vector<std::vector<uint8_t>> sent;
    for (int len : {513, 25, 1, 513, 200}) {
        std::vector<uint8_t> s(len);
        for (auto& v : s) v = rng() & 0xFF;
        s[0] = 0;
        sent.push_back(s);
        appendLane(line, encode(s, len == 25 ? 400 : 176, len == 1 ? 100 : 16), 0);
        line.insert(line.end(), 20, 1);  // mark before break
    }
    DmxFrameParser p;
    std::vector<std::vector<uint8_t>> got;
    for (uint32_t w : receive(line)) {
        if (p.feed(w)) got.emplace_back(p.slots(), p.slots() + p.slotCount());
    }
    if (p.flush()) got.emplace_back(p.slots(), p.slots() + p.slotCount());
    CHECK(got.size() == sent.size(), "got %zu packets, want %zu", got.size(), sent.size());
    for (size_t i = 0; i < got.size() && i < sent.size(); i++) {
        CHECK(got[i] == sent[i], "packet %zu differs (len %zu vs %zu)", i, got[i].size(), sent[i].size());
    }
    CHECK(p.stats().framingErrors == 0 && p.stats().dropped == 0, "unexpected errors");
}

static void testJoinMidPacketAndNoise() {
    std::vector<uint8_t> a(513, 0x55), b(100, 0xAA);
    a[0] = b[0] = 0;
    std::vector<int> line;
    std::vector<uint32_t> wa = encode(a);
    // Start listening halfway through a packet: that partial packet is ignored.
    std::vector<uint32_t> half(wa.begin() + wa.size() / 2, wa.end());
    appendLane(line, half, 0);
    line.insert(line.end(), 10, 1);
    appendLane(line, encode(b), 0);
    // A noisy byte (framing error, non-zero) inside the next packet drops it.
    std::vector<int> noisy;
    appendLane(noisy, encode(a), 0);
    // Corrupt the first stop bit of slot 5 (after break+MAB = 48 words).
    const size_t slot5 = (44 + 4 + 5 * 11) * 4;
    for (int i = 0; i < 4; i++) noisy[slot5 + 9 * 4 + i] = 0;
    line.insert(line.end(), noisy.begin(), noisy.end());
    line.insert(line.end(), 10, 1);
    appendLane(line, encode(b), 0);

    DmxFrameParser p;
    int packets = 0;
    for (uint32_t w : receive(line)) {
        if (p.feed(w)) {
            packets++;
            CHECK(p.slotCount() == 100 && p.slots()[1] == 0xAA, "wrong packet delivered");
        }
    }
    if (p.flush()) {
        packets++;
        CHECK(p.slotCount() == 100, "flushed packet length %u", p.slotCount());
    }
    CHECK(packets == 2, "delivered %d packets, want 2 (partial and noisy ones dropped)", packets);
    CHECK(p.stats().breaks == 3, "breaks %u, want 3 (none lost to the mis-synchronised join)", p.stats().breaks);
    CHECK(p.stats().dropped == 1, "dropped %u, want 1", p.stats().dropped);
}

static void testOverlong() {
    DmxFrameParser p;
    p.feed(kRxBreak);
    for (int i = 0; i < 600; i++) p.feed(i & 0xFF);
    CHECK(p.feed(kRxBreak), "break must close the packet");
    CHECK(p.slotCount() == 513 && p.stats().overlong == 1, "overlong handling");
}

int main() {
    testPacketsOfVaryingLength();
    testJoinMidPacketAndNoise();
    testOverlong();
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("all parser tests passed\n");
    return 0;
}
