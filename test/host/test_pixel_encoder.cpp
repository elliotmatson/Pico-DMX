// Host test for PixelFrameEncoder: decode every lane back from the words.
//
//   c++ -std=c++17 -I src test/host/test_pixel_encoder.cpp src/PixelFrameEncoder.cpp -o /tmp/px && /tmp/px

#include <cstdio>
#include <random>
#include <vector>

#include "PixelFrameEncoder.h"

using namespace pixelstream;

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

int main() {
    std::mt19937 rng(5);
    constexpr int kLanes = 12;
    std::vector<std::vector<uint8_t>> data(kLanes);
    Lane lanes[kLanes];
    const int counts[kLanes] = {2040, 3, 0, 1, 2048, 600, 2040, 9, 2040, 0, 300, 2040};
    for (int i = 0; i < kLanes; i++) {
        data[i].resize(counts[i]);
        for (auto& v : data[i]) v = rng() & 0xFF;
        lanes[i].bytes = counts[i] ? data[i].data() : nullptr;
        lanes[i].count = counts[i];
    }
    std::vector<uint16_t> w(frameWords(2048));
    const size_t n = encodeFrame(w.data(), w.size(), lanes, kLanes);
    CHECK(n == frameWords(2048), "frame words %zu", n);
    uint16_t mask = 0;
    for (int i = 0; i < kLanes; i++) mask |= counts[i] ? 1u << i : 0;
    CHECK(w[0] == mask, "mask %04x want %04x", w[0], mask);
    CHECK(w[1] == 2048 * 8 - 1, "bit count");
    for (int i = 0; i < kLanes; i++) {
        for (size_t b = 0; b < 2048; b++) {
            uint8_t v = 0;
            for (int bit = 0; bit < 8; bit++) v = (uint8_t)((v << 1) | ((w[2 + b * 8 + bit] >> i) & 1));  // MSB first
            const uint8_t want = b < (size_t)counts[i] ? data[i][b] : 0;
            if (v != want) {
                CHECK(false, "lane %d byte %zu = %02x want %02x", i, b, v, want);
                break;
            }
        }
    }
    Lane none[2];
    CHECK(encodeFrame(w.data(), w.size(), none, 2) == 0, "no data: nothing to send");
    CHECK(encodeFrame(w.data(), 100, lanes, kLanes) == 0, "too small a buffer is refused");
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("all pixel encoder tests passed\n");
    return 0;
}
