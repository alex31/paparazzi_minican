/**
 * @file I2C_periph.cpp
 * @brief Shared I2C initialization and recovery logic.
 */

#include "I2C_periph.hpp"
#include "hardwareConf.hpp"
#include "resourceManager.hpp"
#include "UAVCAN/persistantParam.hpp"
#include "stdutil.h"

namespace {
  /// Encode the I2C digital noise filter value into CR1.
  constexpr uint32_t  STM32_CR1_DNF(uint32_t n) {
    return (n << I2C_CR1_DNF_Pos) & I2C_CR1_DNF_Msk;
  }

  static constexpr I2CConfig i2ccfg_100  = {
    .timingr = 0x80C24963, // PCLK1 170Mhz, STANDARD(100 khz), DNF(6), RISE 400ns FALL 200ns 
    .cr1 = STM32_CR1_DNF(6), // Digital noise filter activated (timingr should be aware of that)
    .cr2 = 0, // Only the ADD10 bit can eventually be specified here (10-bit addressing mode)
  };

  static constexpr I2CConfig i2ccfg_400  = {
    .timingr = 0x30C1163F, // PCLK1 170Mhz, FAST(400 khz), DNF(3), RISE 200ns FALL 100ns 
    .cr1 = STM32_CR1_DNF(3), // Digital noise filter activated (timingr should be aware of that)
    .cr2 = 0, // Only the ADD10 bit can eventually be specified here (10-bit addressing mode)
  };

  static constexpr I2CConfig i2ccfg_1000  = {
    .timingr = 0x10E31025, // PCLK1 170Mhz, FAST_PLUS(1 mhz), DNF(1), RISE 120ns FALL 100ns 
    .cr1 = STM32_CR1_DNF(1), // Digital noise filter activated (timingr should be aware of that)
    .cr2 = 0, // Only the ADD10 bit can eventually be specified here (10-bit addressing mode)
  };

  /// Track whether the I2C peripheral has already been started.
  bool started = false;

  /// RAII helper to save/restore GPIO configuration during bus recovery.
  class gpio_config_t {
    using gpio_t = GPIO_TypeDef *;
    const gpio_t gpio_port_a = reinterpret_cast<GPIO_TypeDef *>(GPIOA_BASE);
    const gpio_t gpio_port_b = reinterpret_cast<GPIO_TypeDef *>(GPIOB_BASE);
    GPIO_TypeDef a, b;

  public:
    gpio_config_t() { save(); }

    void restore() {
      *gpio_port_a = a;
      *gpio_port_b = b;
    }

  private:
    void save() {
      a = *gpio_port_a;
      b = *gpio_port_b;
    }
  };

  /// Attempt to unhang a stuck I2C bus by toggling SCL.
  bool i2cUnhangBus()
  {
    bool sdaReleased;
    gpio_config_t context;
    const ioline_t sdaLine = LINE_I2C_SDA;
    const ioline_t sclLine = LINE_I2C_SCL;

    palSetLineMode(sdaLine, PAL_MODE_INPUT);
    chThdSleepMicroseconds(100);
    sdaReleased = palReadLine(sdaLine) == PAL_HIGH;
    uint32_t currentInput;
    if (sdaReleased) {
      context.restore();
      return true;
    }

    palSetLineMode(sclLine, PAL_MODE_INPUT);
    chThdSleepMicroseconds(100);
    currentInput = palReadLine(sclLine) == PAL_HIGH;
    palSetLineMode(sclLine, PAL_MODE_OUTPUT_PUSHPULL);
    palWriteLine(sclLine, currentInput);
    chThdSleepMicroseconds(100);

    for (uint8_t i = 0; i <= 8; i++) {
      chSysPolledDelayX(US2RTC(STM32_SYSCLK, 10)); // 10us: 100 kHz
      palToggleLine(sclLine);
      chSysPolledDelayX(US2RTC(STM32_SYSCLK, 10));
      palToggleLine(sclLine);
      chSysPolledDelayX(US2RTC(STM32_SYSCLK, 10));
      sdaReleased = palReadLine(sdaLine) == PAL_HIGH;
      if (sdaReleased) {
        break;
      }
    }

    context.restore();
    return sdaReleased;
  }

  /// Enable/disable I2C pull-up resistors based on persistent parameter.
  void i2cActivatePullup()
  {
    if (param_cget<"bus.i2c.pullup_resistor">()) {
      palSetLineMode(LINE_PULLUP_SCL, PAL_MODE_OUTPUT_PUSHPULL);
      palSetLineMode(LINE_PULLUP_SDA, PAL_MODE_OUTPUT_PUSHPULL);
      palSetLine(LINE_PULLUP_SCL);
      palSetLine(LINE_PULLUP_SDA);
    } else {
      palSetLineMode(LINE_PULLUP_SCL, PAL_MODE_INPUT);
      palSetLineMode(LINE_PULLUP_SDA, PAL_MODE_INPUT);
    }
  }
}


namespace I2CPeriph
{
  /** @brief Start the shared I2C peripheral with configured timing. */
  DeviceStatus start()
  {
    // already started : nothing to do
    if (started)
      return DeviceStatus(DeviceStatus::I2C);

    using HR = HWResource;
    
    if (not boardResource.tryAcquire(HR::PA15, HR::PB07, HR::I2C_1)) {
      return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT,
			  std::to_underlying(HR::I2C_1));
    }
    i2cActivatePullup();
    
    const uint32_t freqKhz = param_cget<"bus.i2c.frequency_khz">();
    
    if (freqKhz < 400)
      i2cStart(&ExternalI2CD, &i2ccfg_100);
    else if (freqKhz < 1000)
      i2cStart(&ExternalI2CD, &i2ccfg_400);
    else if (freqKhz == 1000)
      i2cStart(&ExternalI2CD, &i2ccfg_1000);
    else
      return DeviceStatus(DeviceStatus::I2C, DeviceStatus::I2C_FREQ_INVALID);

    started = true;
    return DeviceStatus(DeviceStatus::I2C);
  }
  
  /** @brief Stop and reinitialize the I2C peripheral to recover from errors. */
  void reset()
  {
    const auto config = ExternalI2CD.config;
    i2cStop(&ExternalI2CD);
    if (i2cUnhangBus() == false) {
      DebugTrace("unhang bus I2C1 failed");
    }
    
    i2cStart(&ExternalI2CD, config);
  }
};



