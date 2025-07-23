#pragma once
#include "led2812.hpp"

namespace RgbLed {
  void start();
  void setColor(const RGB &rgb);
  void setColor(const HSV &hsv);
  void lightOn();
  void lightOff();
  void lightToggle();
  void setMotif(uint16_t periodMs, uint16_t motif);
  void setWheelOfDeath();
  void setNodeId(uint8_t id);
};
