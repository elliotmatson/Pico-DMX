// Host test for DmxFrameEncoder: decode every lane of an encoded buffer the
// way a DMX receiver would and compare against the source data.
//
//   c++ -std=c++17 -I src test/host/test_frame_encoder.cpp src/DmxFrameEncoder.cpp -o /tmp/t && /tmp/t

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

#include "DmxFrameEncoder.h"

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

struct Decoded {
    uint32_t breakWords = 0;
    uint32_t mabWords = 0;
    std::vector<uint8_t> slots;
    int framingErrors = 0;
};

// Walk one lane's line level word by word like a UART receiver.
static Decoded decodeLane(const std::vector<uint32_t>& words, int bit) {
    Decoded d;
    auto level = [&](size_t w) { return w < words.size() ? (words[w] >> bit) & 1u : 1u; };
    size_t w = 0;
    while (w < words.size() && level(w) == 0) {
        d.breakWords++, w++;
    }
    while (w < words.size() && level(w) == 1) {
        d.mabWords++, w++;
    }
    while (w < words.size()) {
        if (level(w) == 1) {
            w++;
            continue;  // idle / mark between slots
        }
        uint8_t v = 0;
        for (int b = 0; b < 8; b++) {
            v |= level(w + 1 + b) << b;
        }
        if (level(w + 9) != 1 || level(w + 10) != 1) {
            d.framingErrors++;
        }
        d.slots.push_back(v);
        w += kWordsPerSlot;
    }
    return d;
}

static void testTwelveLanesWithDe() {
    std::mt19937 rng(1234);
    constexpr int kLanes = 12;
    std::vector<std::vector<uint8_t>> data(kLanes, std::vector<uint8_t>(kMaxSlots));
    Lane lanes[kLanes];
    for (int i = 0; i < kLanes; i++) {
        for (auto& v : data[i]) v = rng() & 0xFF;
        data[i][0] = 0x00;
        lanes[i].enabled = true;
        lanes[i].dataBit = i;
        lanes[i].deBit = 12 + i;
        lanes[i].slots = data[i].data();
        lanes[i].slotCount = kMaxSlots;
    }
    // Per-lane variations: a short universe, long break, long MAB, disabled lane.
    lanes[3].slotCount = 25;
    lanes[5].breakUs = 400;
    lanes[6].mabUs = 100;
    lanes[9].enabled = false;
    lanes[10].breakUs = 50;  // below minimum: must clamp to 92 us
    lanes[11].mabUs = 4;     // below minimum: must clamp to 12 us

    std::vector<uint32_t> buf(maxFrameWords());
    const size_t n = encodeFrame(buf.data(), buf.size(), lanes, kLanes);
    buf.resize(n);
    CHECK(n == laneWords(lanes[5]), "frame length %zu != longest lane %u", n, laneWords(lanes[5]));

    for (int i = 0; i < kLanes; i++) {
        const Decoded d = decodeLane(buf, i);
        if (!lanes[i].enabled) {
            CHECK(d.breakWords == 0 && d.slots.empty(), "disabled lane %d must idle at mark", i);
            for (uint32_t w : buf) CHECK(!((w >> (12 + i)) & 1u), "disabled lane %d DE must be low", i);
            continue;
        }
        const uint32_t expBreak = (lanes[i].breakUs < kMinBreakUs ? kMinBreakUs : lanes[i].breakUs + 3) / 4;
        const uint32_t expMab = ((lanes[i].mabUs < kMinMabUs ? kMinMabUs : lanes[i].mabUs) + 3) / 4;
        CHECK(d.breakWords == expBreak, "lane %d break %u words, want %u", i, d.breakWords, expBreak);
        // A start code of 0x00 begins with a 0 start bit, so MAB is exact.
        CHECK(d.mabWords == expMab, "lane %d MAB %u words, want %u", i, d.mabWords, expMab);
        CHECK(d.framingErrors == 0, "lane %d framing errors %d", i, d.framingErrors);
        CHECK(d.slots.size() == lanes[i].slotCount, "lane %d slots %zu, want %u", i, d.slots.size(),
              lanes[i].slotCount);
        for (size_t s = 0; s < d.slots.size() && s < lanes[i].slotCount; s++) {
            if (d.slots[s] != data[i][s]) {
                CHECK(false, "lane %d slot %zu = %u, want %u", i, s, d.slots[s], data[i][s]);
                break;
            }
        }
        for (uint32_t w : buf) {
            if (!((w >> (12 + i)) & 1u)) {
                CHECK(false, "lane %d DE dropped during frame", i);
                break;
            }
        }
    }
    // The frame must end with every data line at mark so the stall is idle.
    CHECK((buf.back() & 0xFFFu) == 0xFFFu, "last word not idle: %08x", buf.back());
}

static void testCapacityAndEmpty() {
    Lane l;
    l.enabled = true;
    l.dataBit = 0;
    uint8_t slots[513] = {};
    l.slots = slots;
    l.slotCount = 513;
    uint32_t small[10];
    CHECK(encodeFrame(small, 10, &l, 1) == 0, "must refuse a buffer that is too small");

    Lane off;
    off.dataBit = 0;
    uint32_t one[1];
    CHECK(encodeFrame(one, 1, &off, 1) == 1 && one[0] == 1u, "all-disabled must give one idle word");
    uint32_t pad[300];
    const size_t n = encodeFrame(pad, 300, &off, 1, 250);
    CHECK(n == 250 && pad[249] == 1u && detail::encodeFramePerLane(pad, 300, &off, 1, 250) == 250,
          "all-disabled pads to minWords on both paths");
}

// Uniform timing takes the fast path; it must match the per-lane path exactly
// and decode correctly, including short lanes, a disabled lane and DE bits in
// another byte group.
static void testFastPathMatchesPerLane() {
    std::mt19937 rng(99);
    constexpr int kLanes = 12;
    std::vector<std::vector<uint8_t>> data(kLanes, std::vector<uint8_t>(kMaxSlots));
    Lane lanes[kLanes];
    for (int i = 0; i < kLanes; i++) {
        for (auto& v : data[i]) v = rng() & 0xFF;
        data[i][0] = 0x00;
        lanes[i].enabled = true;
        lanes[i].dataBit = (i < 8) ? i : 16 + (i - 8);  // gap to exercise byte groups
        lanes[i].deBit = (i < 8) ? 8 + i : 24 + (i - 8);
        lanes[i].slots = data[i].data();
        lanes[i].slotCount = kMaxSlots;
    }
    lanes[2].slotCount = 1;    // start code only
    lanes[7].slotCount = 100;
    lanes[10].enabled = false;

    std::vector<uint32_t> fast(maxFrameWords()), slow(maxFrameWords());
    const size_t nf = encodeFrame(fast.data(), fast.size(), lanes, kLanes);
    const size_t ns = detail::encodeFramePerLane(slow.data(), slow.size(), lanes, kLanes);
    CHECK(nf == ns && nf > 0, "lengths differ: fast %zu, per-lane %zu", nf, ns);
    for (size_t w = 0; w < nf && w < ns; w++) {
        if (fast[w] != slow[w]) {
            CHECK(false, "word %zu differs: fast %08x, per-lane %08x", w, fast[w], slow[w]);
            break;
        }
    }
    fast.resize(nf);
    for (int i = 0; i < kLanes; i++) {
        if (!lanes[i].enabled) continue;
        const Decoded d = decodeLane(fast, lanes[i].dataBit);
        CHECK(d.slots.size() == lanes[i].slotCount && d.framingErrors == 0, "lane %d decode", i);
        for (size_t s = 0; s < d.slots.size(); s++) {
            if (d.slots[s] != data[i][s]) {
                CHECK(false, "lane %d slot %zu mismatch", i, s);
                break;
            }
        }
    }
}

// RDM: a lane that releases DE drops it right after its packet; padding
// stretches the frame; both paths agree.
static void testReleaseDeAndPadding() {
    std::vector<uint8_t> dmx(kMaxSlots, 0x11), rdm(26, 0x55);
    dmx[0] = 0x00;
    rdm[0] = 0xCC;
    Lane lanes[2];
    lanes[0].enabled = lanes[1].enabled = true;
    lanes[0].dataBit = 0, lanes[0].deBit = 8, lanes[0].slots = dmx.data(), lanes[0].slotCount = 100;
    lanes[1].dataBit = 1, lanes[1].deBit = 9, lanes[1].slots = rdm.data(), lanes[1].slotCount = 26;
    lanes[1].releaseDe = true;
    const size_t pad = 4000;
    std::vector<uint32_t> a(maxFrameWords()), b(maxFrameWords());
    const size_t na = encodeFrame(a.data(), a.size(), lanes, 2, pad);
    const size_t nb = detail::encodeFramePerLane(b.data(), b.size(), lanes, 2, pad);
    CHECK(na == pad && nb == pad, "padded to %zu / %zu", na, nb);
    CHECK(std::equal(a.begin(), a.begin() + na, b.begin()), "fast and per-lane paths agree");
    const size_t end = laneWords(lanes[1]);
    CHECK(((a[end - 1] >> 9) & 1) && !((a[end] >> 9) & 1) && !((a[na - 1] >> 9) & 1), "DE drops at word %zu", end);
    CHECK(((a[na - 1] >> 8) & 1), "the DMX lane keeps DE for the whole frame");
    CHECK(((a[na - 1] >> 1) & 1), "the RDM lane idles at mark after its packet");
}

// A lane sitting a frame out keeps its driver on with holdDe, and drops it
// without; neither sends data. A frame of only holding lanes still holds DE.
static void testHoldDe() {
    std::vector<uint8_t> dmx(kMaxSlots, 0x00);
    Lane lanes[3];
    lanes[0].enabled = true;
    lanes[0].dataBit = 0, lanes[0].deBit = 8, lanes[0].slots = dmx.data(), lanes[0].slotCount = 10;
    lanes[1].dataBit = 1, lanes[1].deBit = 9, lanes[1].holdDe = true;
    lanes[2].dataBit = 2, lanes[2].deBit = 10;
    std::vector<uint32_t> a(maxFrameWords()), b(maxFrameWords());
    const size_t na = encodeFrame(a.data(), a.size(), lanes, 3);
    const size_t nb = detail::encodeFramePerLane(b.data(), b.size(), lanes, 3);
    CHECK(na == nb && std::equal(a.begin(), a.begin() + na, b.begin()), "fast and per-lane paths agree");
    bool held = true, mark = true, dropped = true;
    for (size_t i = 0; i < na; i++) {
        held = held && ((a[i] >> 9) & 1);
        mark = mark && ((a[i] >> 1) & 1) && ((a[i] >> 2) & 1);
        dropped = dropped && !((a[i] >> 10) & 1);
    }
    CHECK(held, "the holding lane keeps DE for the whole frame");
    CHECK(mark, "disabled lanes stay at mark");
    CHECK(dropped, "a disabled lane without holdDe drops DE");

    lanes[0].enabled = false;
    const size_t n = encodeFrame(a.data(), a.size(), lanes, 3, 250);
    CHECK(n == 250 && ((a[0] >> 9) & 1) && ((a[n - 1] >> 9) & 1) && !((a[0] >> 8) & 1),
          "an idle frame holds DE only on the holding lane");
}

int main() {
    testHoldDe();
    testTwelveLanesWithDe();
    testReleaseDeAndPadding();
    testFastPathMatchesPerLane();
    testCapacityAndEmpty();
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("all encoder tests passed\n");
    return 0;
}
