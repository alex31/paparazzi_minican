#pragma once
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>

using systime_t = uint32_t;
using sysinterval_t = uint32_t;
constexpr systime_t TIME_MS2I(unsigned ms) { return ms * 20U; }
constexpr systime_t TIME_S2I(unsigned seconds) { return TIME_MS2I(seconds * 1000U); }
constexpr systime_t chTimeDiffX(systime_t start, systime_t end) { return end - start; }
systime_t chVTGetSystemTimeX();
void chThdSleep(sysinterval_t);
void chRegSetThreadName(const char*);
void chSysLock();
void chSysUnlock();
void chThdCreateStatic(void*, size_t, unsigned, void (*)(void*), void*);
#define THD_WORKING_AREA(name, size) uint8_t name[size]
#define DMA_SECTION ".test_dma"
#define FAST_SECTION ".test_fast"
constexpr unsigned NORMALPRIO = 128;
#define RGBLED_TIM 1
#define RGBLED_TIM_CH 1
constexpr uint32_t STM32_DMAMUX1_TIM1_UP = 1;
constexpr unsigned STM32_DMA_STREAM_ID_ANY = 0;
struct PWMDriver {};
inline PWMDriver PWMD1;
struct LedTiming { unsigned t0h, t1h; };
constexpr LedTiming getClockByTimer(PWMDriver*) { return {1, 2}; }
enum class TimerChannel { CH1 };
struct RGB { uint8_t r, g, b; };
struct HSV { float h, s, v; };
#include "color_helpers.hpp"
struct LedPixel {
    bool isHsv = false;
    RGB rgb{};
    HSV hsv{};
    void setRGB(RGB value) { isHsv = false; rgb = value; }
    void setHSV(HSV value) { isHsv = true; hsv = value; }
};
void recordFrame(const LedPixel&);
template <class T, unsigned H0, unsigned H1> struct Led2812 {};
template <size_t N, class Led> struct Led2812Strip {
    LedPixel pixel;
    Led2812Strip(PWMDriver*, LedTiming, unsigned, uint32_t, TimerChannel) {}
    LedPixel& operator[](size_t index) { assert(index == 0); return pixel; }
    void emitFrame() { recordFrame(pixel); }
};
