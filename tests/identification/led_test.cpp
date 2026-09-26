// Tests the real rgbLeds.cpp with a simulated clock and LED transport.
#include "rgbLeds.hpp"
#include <cassert>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

struct Frame { unsigned ticks; RGB color; };
static std::vector<Frame> frames;
static std::vector<unsigned> polls;
static unsigned elapsed, limit, enableAt, disableAt, transportTicks, sleeps;
static systime_t initialTime;
static bool identification, toggle, wheel;
static void (*worker)(void*);
struct Finished {};

bool same(RGB a, RGB b) { return a.r == b.r && a.g == b.g && a.b == b.b; }
constexpr RGB black{0, 0, 0};
constexpr RGB purple = hsv2rgb(HSV{0.8f, 1, 0.5f});
constexpr RGB red = hsv2rgb(HSV{0, 1, 0.1f});
constexpr RGB yellow = hsv2rgb(HSV{0.166f, 1, 0.1f});
constexpr RGB green = hsv2rgb(HSV{0.333f, 1, 0.1f});

void advance(unsigned ticks) {
    const unsigned end = elapsed + ticks;
    // Writes can happen between two wakes, including just after a parameter read.
    if (toggle && elapsed < enableAt && end >= enableAt) identification = true;
    if (toggle && elapsed < disableAt && end >= disableAt) identification = false;
    if (wheel && elapsed < enableAt && end >= enableAt) RgbLed::setWheelOfDeath();
    elapsed = end;
    if (elapsed >= limit) throw Finished{};
}
systime_t chVTGetSystemTimeX() { return initialTime + elapsed; }
void chRegSetThreadName(const char*) {}
void chSysLock() {}
void chSysUnlock() {}
void chThdCreateStatic(void*, size_t, unsigned, void (*function)(void*), void*) { worker = function; }
void chThdSleep(sysinterval_t ticks) {
    assert(ticks > 0 && ticks <= TIME_MS2I(200));
    ++sleeps;
    advance(ticks);
}
void recordFrame(const LedPixel& pixel) {
    assert(!pixel.isHsv); // RGB is compared before encoding/submitting a frame.
    if (!frames.empty()) assert(!same(frames.back().color, pixel.rgb));
    frames.push_back({elapsed, pixel.rgb});
    advance(transportTicks);
}
bool requestIdentification() { polls.push_back(elapsed); return identification; }

RGB at(unsigned ticks) {
    // The WS2812 retains its last color between transmissions.
    RGB color = black;
    for (const auto& frame : frames) {
        if (frame.ticks > ticks) break;
        color = frame.color;
    }
    return color;
}
unsigned firstAfter(unsigned ticks, RGB color) {
    for (const auto& frame : frames) {
        if (frame.ticks >= ticks && same(frame.color, color)) return frame.ticks;
    }
    assert(false);
    return 0;
}
void expect(unsigned ticks, RGB color) { assert(same(at(ticks), color)); }

int main(int argc, char** argv) {
    assert(argc >= 2);
    const std::string scenario(argv[1]);
    limit = TIME_MS2I(3500);
    if (scenario == "motif" || scenario == "minimal" || scenario.starts_with("constant")) {
        // Boot identification and the bootloader install no live reader.
        RgbLed::setColor(RGB{9, 8, 7});
        const uint16_t pattern = scenario == "constant-off" ? 0 :
            scenario.starts_with("constant") ? 0xffff : 0b1010100000000000;
        RgbLed::setMotif(150, pattern);
        if (scenario == "minimal" || scenario == "constant-minimal") RgbLed::startMinimal();
        else RgbLed::start();
    } else {
        RgbLed::start();
        RgbLed::setNodeId(38, requestIdentification); // base-5 digits 1, 2, 3
        if (scenario == "toggle") {
            assert(argc == 3);
            enableAt = TIME_MS2I(static_cast<unsigned>(std::stoi(argv[2])));
            disableAt = enableAt + TIME_MS2I(1300);
            limit = disableAt + TIME_MS2I(1200);
            toggle = true;
        } else if (scenario == "initially-on") {
            identification = true;
            toggle = true;
            enableAt = TIME_MS2I(10000);
            disableAt = TIME_MS2I(250);
            limit = TIME_MS2I(1400);
        } else if (scenario == "identification") {
            identification = true;
        } else if (scenario == "wrap") {
            initialTime = std::numeric_limits<systime_t>::max() - TIME_MS2I(600);
        } else if (scenario == "transport") {
            transportTicks = TIME_MS2I(3); // Emitting frames must not shift deadlines.
        } else if (scenario == "wheel") {
            identification = true;
            enableAt = TIME_MS2I(100);
            wheel = true;
            limit = TIME_MS2I(250);
        } else assert(scenario == "digits");
    }
    try { worker(nullptr); assert(false); } catch (const Finished&) {}
    for (size_t i = 1; i < polls.size(); ++i) {
        assert(polls[i] - polls[i - 1] <= TIME_MS2I(200));
    }
    if (scenario == "motif" || scenario == "minimal") {
        assert(polls.empty());
        expect(0, black);
        expect(TIME_MS2I(1649), black);
        expect(TIME_MS2I(1650), RGB{9, 8, 7});
        expect(TIME_MS2I(1800), black);
        expect(TIME_MS2I(1950), RGB{9, 8, 7});
        expect(TIME_MS2I(2100), black);
        expect(TIME_MS2I(2250), RGB{9, 8, 7});
        expect(TIME_MS2I(2400), black);
        assert(frames.size() == 7 && sleeps <= 24);
    } else if (scenario.starts_with("constant")) {
        assert(polls.empty() && frames.size() == 1 && sleeps == 18);
        expect(0, scenario == "constant-off" ? black : RGB{9, 8, 7});
    } else if (scenario == "toggle" || scenario == "initially-on") {
        const unsigned begin = scenario == "toggle" ? firstAfter(enableAt, purple) : 0;
        if (scenario == "toggle") assert(begin - enableAt <= TIME_MS2I(200));
        expect(begin, purple);
        expect(begin + TIME_MS2I(150), black);
        if (scenario == "toggle") {
            expect(begin + TIME_MS2I(300), purple);
            expect(begin + TIME_MS2I(450), black);
            expect(begin + TIME_MS2I(600), purple);
            expect(begin + TIME_MS2I(750), black);
        }
        const auto resumed = firstAfter(disableAt, red);
        assert(resumed - disableAt <= TIME_MS2I(200));
        expect(resumed + TIME_MS2I(500), black);
        expect(resumed + TIME_MS2I(800), yellow);
        assert(polls.size() < limit / TIME_MS2I(100));
    } else if (scenario == "wheel") {
        expect(0, purple);
        const auto resumed = firstAfter(enableAt, hsv2rgb(HSV{0, 1, 0.2f}));
        assert(resumed - enableAt <= TIME_MS2I(200));
        expect(resumed + TIME_MS2I(10), hsv2rgb(HSV{0.03f, 1, 0.2f}));
    } else if (scenario == "identification") {
        expect(0, purple);
        expect(TIME_MS2I(150), black);
        expect(TIME_MS2I(300), purple);
        expect(TIME_MS2I(600), purple);
        expect(TIME_MS2I(750), black);
        expect(TIME_MS2I(2399), black);
        expect(TIME_MS2I(2400), purple);
        assert(frames.size() == 12 && polls.size() <= 26);
    } else {
        expect(0, red);
        expect(TIME_MS2I(499), red);
        expect(TIME_MS2I(500), black);
        expect(TIME_MS2I(800), yellow);
        expect(TIME_MS2I(1300), black);
        expect(TIME_MS2I(1600), green);
        expect(TIME_MS2I(2100), black);
        expect(TIME_MS2I(3399), black);
        expect(TIME_MS2I(3400), red);
        assert(polls.size() == 21 && frames.size() == 7);
    }
    std::printf("LED %s%s%s: OK (%zu frames, %u sleeps)\n", argv[1],
                argc == 3 ? " at " : "", argc == 3 ? argv[2] : "", frames.size(), sleeps);
}
