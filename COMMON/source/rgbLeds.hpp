/**
 * @file rgbLeds.hpp
 * @brief API for the onboard WS2812 status LED.
 */
#pragma once
#include "led2812.hpp"

namespace RgbLed {
  /** @brief Start the full LED animation thread. */
  void start();
  /** @brief Start a minimal LED animation thread without float use. */
  void startMinimal(); // no use of float for bootloader use
  /** @brief Set the LED to an RGB color. */
  void setColor(const RGB &rgb);
  /** @brief Set the LED to an HSV color. */
  void setColor(const HSV &hsv);
  /** @brief Turn the LED on. */
  void lightOn();
  /** @brief Turn the LED off. */
  void lightOff();
  /** @brief Toggle the LED on/off state. */
  void lightToggle();
  /** @brief Set a bitmask motif and its period. */
  void setMotif(uint16_t periodMs, uint16_t motif);
  /** @brief Enable the "wheel of death" animation. */
  void setWheelOfDeath();
  /** @brief Configure the node ID to display in minimal mode. */
  void setNodeId(uint8_t id);
};
