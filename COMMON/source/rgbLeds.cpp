/**
 * @file rgbLeds.cpp
 * @brief Onboard WS2812 status LED implementation.
 */
#include "rgbLeds.hpp"

/*
 * Concatenate preprocessor tokens A and B after macro-expanding them.
 */
#define CONCAT_NX(st1, st2) st1 ## st2
#define CONCAT3_NX(st1, st2, st3) st1 ## st2 ## st3
#define CONCAT(st1, st2) CONCAT_NX(st1, st2)
#define CONCAT3(st1, st2, st3) CONCAT3_NX(st1, st2, st3)

static constexpr PWMDriver &ledPwm =  CONCAT(PWMD, RGBLED_TIM);
static constexpr uint32_t dmaMux = CONCAT3(STM32_DMAMUX1_TIM, RGBLED_TIM, _UP);
static constexpr LedTiming ledTiming = getClockByTimer(&ledPwm);
using Led_t = Led2812<uint16_t, ledTiming.t0h, ledTiming.t1h> ;

namespace {
  __attribute__ ((section(DMA_SECTION "_clear"), aligned(4)))
  Led2812Strip<1, Led_t> leds(&ledPwm, ledTiming,
			      STM32_DMA_STREAM_ID_ANY,
			      dmaMux,
			      static_cast<TimerChannel>(RGBLED_TIM_CH - 1U));
  
  THD_WORKING_AREA(waLedsAnim, 512) __attribute__((section(FAST_SECTION "_clear")));	
  void  ledsAnim(void *arg);
  void  ledsAnimMinimal(void *arg);
  sysinterval_t displayId(bool restart, systime_t now, RGB& color);
  bool (* volatile identificationRequested)() = nullptr;
  RGB   rgb;
  volatile bool off = false;
  volatile bool wheelOfDeathMode = false;
  volatile systime_t periodI = TIME_S2I(10);
  volatile uint16_t motif = 0xffff;
  volatile struct {
    uint8_t digits[3];
  } nodeId = {};
  
}

/** @brief Start the full LED animation thread. */
void RgbLed::start()
{
  chThdCreateStatic(waLedsAnim, sizeof(waLedsAnim), NORMALPRIO, &ledsAnim, NULL);
}

/** @brief Start the minimal LED animation thread. */
void RgbLed::startMinimal()
{
  chThdCreateStatic(waLedsAnim, sizeof(waLedsAnim), NORMALPRIO, &ledsAnimMinimal, NULL);
}

/** @brief Set the LED color using RGB input. */
void RgbLed::setColor(const RGB &_rgb)
{
  rgb = _rgb;
}

/** @brief Set the LED color using HSV input. */
void RgbLed::setColor(const HSV &hsv)
{
  rgb = hsv2rgb(hsv);
}

/** @brief Turn the LED off. */
void RgbLed::lightOff()
{
  off = true;
}

/** @brief Turn the LED on. */
void RgbLed::lightOn()
{
  off = false;
}

/** @brief Toggle the LED on/off state. */
void RgbLed::lightToggle()
{
  off = not off;
}

/** @brief Enable the "wheel of death" animation. */
void RgbLed::setWheelOfDeath()
{
  wheelOfDeathMode = true;
}

/** @brief Store the node ID digits for display. */
void RgbLed::setNodeId(uint8_t id, bool (*checkIdentification)())
{
  id = std::min(id, static_cast<uint8_t>(124U));
  chSysLock();
  identificationRequested = checkIdentification;
  nodeId.digits[0] = id / 25U;
  nodeId.digits[1] = (id / 5U) % 5U;
  nodeId.digits[2] = id % 5U;
  chSysUnlock();
}
 
/** @brief Configure the motif period and bitmask. */
void RgbLed::setMotif(uint16_t _periodMs, uint16_t _motif)
{
  periodI = TIME_MS2I(_periodMs);
  motif = _motif;
}

namespace {
  constexpr sysinterval_t maxPollInterval = TIME_MS2I(200);
  constexpr RGB black{0, 0, 0};
  constexpr RGB identificationColor = hsv2rgb(HSV{0.8f, 1, 0.5f});

  /** @brief The LED retains its color; only changed colors need a DMA frame. */
  struct LedOutput {
    RGB previous{};
    bool initialized = false;

    void show(const RGB& color) {
      if (!initialized || color.r != previous.r || color.g != previous.g || color.b != previous.b) {
        leds[0].setRGB(color);
        leds.emitFrame();
        previous = color;
        initialized = true;
      }
    }
  };

  /** @brief Advance a bitmask motif and return the time to its next color change. */
  struct MotifAnimation {
    systime_t timestamp;
    uint8_t index = 0;

    sysinterval_t frame(systime_t now, sysinterval_t period, uint16_t pattern,
                        const RGB& onColor, RGB& color) {
      // A zero period must not cause division by zero or a busy loop.
      period = std::max<sysinterval_t>(period, 1U);
      const sysinterval_t elapsed = chTimeDiffX(timestamp, now);
      const auto steps = elapsed / period;
      index = (index + steps) % 16U;
      timestamp += steps * period;
      const bool lit = (pattern & (1U << index)) != 0;
      color = lit ? onColor : black;

      // Skip consecutive identical bits, but keep polling configuration changes.
      unsigned next = 1;
      while (next < 16U && ((pattern & (1U << ((index + next) % 16U))) != 0) == lit) {
        ++next;
      }
      if (next == 16U) return maxPollInterval; // Constant motif.
      return next * period - chTimeDiffX(timestamp, now);
    }
  };

  /** @brief Account for rendering/transport time so LED timing does not drift. */
  void sleepUntilTransition(systime_t frameStart, sysinterval_t remaining) {
    const sysinterval_t interval = std::min(remaining, maxPollInterval);
    const sysinterval_t spent = chTimeDiffX(frameStart, chVTGetSystemTimeX());
    chThdSleep(spent < interval ? interval - spent : 1U);
  }

  /** @brief Full animation loop handling wheel, node ID, and motif display. */
  void ledsAnim(void *arg)
  {
    (void)arg;
    chRegSetThreadName("ledsAnim");
    LedOutput output;
    MotifAnimation statusMotif{chVTGetSystemTimeX()};
    MotifAnimation identificationMotif{statusMotif.timestamp, 11};
    bool showingNodeId = false;
    bool showingIdentification = false;
    while (true) {
      const systime_t now = chVTGetSystemTimeX();
      const auto checkIdentification = identificationRequested;
      const bool identify = checkIdentification && checkIdentification();
      const bool hasNodeId = (nodeId.digits[0] | nodeId.digits[1] | nodeId.digits[2]) != 0;
      RGB color = black;
      sysinterval_t remaining = maxPollInterval;
      if (wheelOfDeathMode) {
        static float hue = 0;
        color = hsv2rgb(HSV{hue, 1, 0.2f});
        hue += 0.03f;
        if (hue > 1.0f) hue = 0;
        remaining = TIME_MS2I(10); // Preserve the fault animation's speed.
      } else if (identify) {
        if (!showingIdentification) identificationMotif = {now, 11};
        remaining = identificationMotif.frame(now, TIME_MS2I(150),
                                             0b1010100000000000, identificationColor, color);
      } else if (hasNodeId) {
        remaining = displayId(!showingNodeId, now, color);
      } else if (!off) {
        remaining = statusMotif.frame(now, periodI, motif, rgb, color);
      }
      showingIdentification = !wheelOfDeathMode && identify;
      showingNodeId = !wheelOfDeathMode && !identify && hasNodeId;
      output.show(color);
      sleepUntilTransition(now, remaining);
    }
  }

  /** @brief Minimal bootloader animation: no float use or parameter reader. */
  void ledsAnimMinimal(void *arg)
  {
    (void)arg;
    chRegSetThreadName("ledsAnim");
    LedOutput output;
    MotifAnimation statusMotif{chVTGetSystemTimeX()};
    while (true) {
      const systime_t now = chVTGetSystemTimeX();
      RGB color;
      const sysinterval_t remaining = statusMotif.frame(now, periodI, motif, rgb, color);
      output.show(color);
      sleepUntilTransition(now, remaining);
    }
  }

  /*
    Blanc ⚪️	(peu importe H, S=0, V=1)
    Rouge 🔴	0.0f
    Jaune 🟡	0.166f (environ 60°)
    Vert  🟢	0.333f (environ 120°)
    Bleu  🔵	0.666f (environ 240°)

    Le node affiche le NodeId en base 5, digit par digit, de gauche à droite.
    
    Pour chaque digit :
    
    Afficher la couleur correspondant au digit pendant 0,5 seconde.
    Éteindre la LED pendant 0,3 seconde pour bien séparer chaque digit.

    Après avoir affiché tous les digits, le NodeId complet est suivi d'une pause plus longue
    (LED éteinte pendant 1 seconde supplémentaire),
    puis la séquence se répète en boucle.

    id = digitA*25 + digitB*5 + digitC
    
   */
  /** @brief Display the current digit and return the time to its next transition. */
  sysinterval_t displayId(bool restart, systime_t now, RGB& color)
  {
    // These conversions are evaluated at compile time, not at every poll.
    static constexpr RGB colors[] = {
      hsv2rgb(HSV{0.001f, 0.001f, 0.1f}), // White: 0
      hsv2rgb(HSV{0, 1, 0.1f}),          // Red: 1
      hsv2rgb(HSV{0.166f, 1, 0.1f}),     // Yellow: 2
      hsv2rgb(HSV{0.333f, 1, 0.1f}),     // Green: 3
      hsv2rgb(HSV{0.666f, 1, 0.1f}),     // Blue: 4
    };
    // Last blank includes the 300 ms separator and the 1 s end-of-ID pause.
    static constexpr sysinterval_t durations[] = {
      TIME_MS2I(500), TIME_MS2I(300), TIME_MS2I(500),
      TIME_MS2I(300), TIME_MS2I(500), TIME_MS2I(1300),
    };
    static uint8_t phase = 0; // Even: show digit, odd: blank between digits.
    static systime_t phaseTs = 0;
    if (restart) {
      phase = 0;
      phaseTs = now;
    }
    while (chTimeDiffX(phaseTs, now) >= durations[phase]) {
      phaseTs += durations[phase];
      phase = (phase + 1U) % 6U;
    }
    color = phase % 2 ? black : colors[nodeId.digits[phase / 2U]];
    return durations[phase] - chTimeDiffX(phaseTs, now);
  }
}
