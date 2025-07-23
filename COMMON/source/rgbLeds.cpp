#include <ch.h>
#include <hal.h>
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
  void  displayId();
  RGB   rgb;
  volatile bool off = false;
  volatile bool wheelOfDeathMode = false;
  volatile systime_t periodI = TIME_S2I(10);
  volatile uint16_t motif = 0xffff;
  volatile uint8_t motifIndex = 0;
  volatile union {
    uint8_t digits[3];
    uint32_t raw;
  } nodeId = {};
  
}

void RgbLed::start()
{
  chThdCreateStatic(waLedsAnim, sizeof(waLedsAnim), NORMALPRIO, &ledsAnim, NULL);
}

void RgbLed::setColor(const RGB &_rgb)
{
  rgb = _rgb;
}

void RgbLed::setColor(const HSV &hsv)
{
  rgb = hsv2rgb(hsv);
}

void RgbLed::lightOff()
{
  off = true;
}

void RgbLed::lightOn()
{
  off = false;
}

void RgbLed::lightToggle()
{
  off = not off;
}

void RgbLed::setWheelOfDeath()
{
  wheelOfDeathMode = true;
}

void RgbLed::setNodeId(uint8_t id)
{
  id = std::min(id, static_cast<uint8_t>(124U));
  nodeId.digits[0] = id / 25U;
  nodeId.digits[1] = (id / 5U) % 5U;
  nodeId.digits[2] = id % 5U;
}
 
void RgbLed::setMotif(uint16_t _periodMs, uint16_t _motif)
{
  periodI = TIME_MS2I(_periodMs);
  motif = _motif;
}

namespace {
  void  ledsAnim (void *arg)	
  {
    (void)arg;					
    chRegSetThreadName("ledsAnim");		
    systime_t ts = chVTGetSystemTimeX();
    while(true) {
      if (wheelOfDeathMode) {
	static float hue = 0;
	leds[0].setHSV(HSV{hue, 1, 0.2f});
	hue = hue + 0.03f;
	if (hue > 1.0f)
	  hue = 0;
      } else if (nodeId.raw != 0) {
	displayId();
      } else if (not off) {
	const systime_t now = chVTGetSystemTimeX();
	if (chTimeDiffX(ts, now) > periodI) {
	  ts = now;
	  leds[0].setRGB(motif & (1U << motifIndex) ? rgb : RGB{0,0,0});
	  motifIndex = (motifIndex + 1U) % (sizeof motif * 8U);
	}
      } else {
	leds[0].setRGB(RGB{0,0,0});
      }
      leds.emitFrame();
      chThdSleepMilliseconds(10);
    }
  }
  
  /*
    Blanc ⚪️	(peu importe H, S=0, V=1)
    Rouge 🔴	0.0f
    Jaune 🟡	0.166f (environ 60°)
    Vert  🟢	0.333f (environ 120°)
    Bleu  🔵	0.666f (environ 240°)

    Le node affiche le NodeId décimal digit par digit, de gauche à droite.
    
    Pour chaque digit :
    
    Afficher la couleur correspondant au digit pendant 0.8 seconde.
    Éteindre la LED pendant 0,3 seconde pour bien séparer chaque digit.

    Après avoir affiché tous les digits, le NodeId complet est suivi d'une pause plus longue
    (LED éteinte pendant 2 secondes),
    puis la séquence se répète en boucle.

    id = digitA*25 + digitB*5 + digitC
    
   */
  void  displayId()
  {
    static constexpr float value = 0.10;
    static const float huesat[5][2] =  {
      {0.001, 0.001},	// ⚪️ 0 
      {0, 1},		// 🔴 1 
      {.166, 1},	// 🟡 2 		
      {.333, 1},	// 🟢 3 
      {.666, 1}		// 🔵 4 
    };

    static auto ledOff = [] {
      leds[0].setRGB(RGB{0,0,0});
      leds.emitFrame();
      chThdSleepMilliseconds(300);
    };

    static auto ledShowDigit = [](size_t i) {
      leds[0].setHSV(HSV{huesat[nodeId.digits[i]][0], huesat[nodeId.digits[i]][1], value});
      leds.emitFrame();
      chThdSleepMilliseconds(500);
      ledOff();
    };

    // show the 3 digits one after another
    for (const auto digitIdx : {0, 1, 2})
      ledShowDigit(digitIdx);

    chThdSleepSeconds(1);
  }

  
}
