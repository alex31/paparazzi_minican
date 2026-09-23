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

  /** @brief Snapshot only one GPIO pad, never a complete shared GPIO port. */
  class gpio_pad_config_t {
  public:
    gpio_pad_config_t(GPIO_TypeDef *gpio, uint32_t index)
      : port(gpio), pad(index), oneBit(1U << index),
	twoBits(3U << (2U * index)), fourBits(15U << (4U * (index & 7U))),
	moder(gpio->MODER & twoBits), otyper(gpio->OTYPER & oneBit),
	ospeedr(gpio->OSPEEDR & twoBits), pupdr(gpio->PUPDR & twoBits),
	alternate(gpio->AFR[index / 8U] & fourBits),
	output(gpio->ODR & oneBit)
    {
    }

    void restore() const
    {
      // Restore the output latch and alternate function before the mode to
      // avoid a glitch. Masked RMW preserves every unrelated GPIO pad.
      port->BSRR = output != 0U ? oneBit : oneBit << 16U;
      port->OTYPER = (port->OTYPER & ~oneBit) | otyper;
      port->OSPEEDR = (port->OSPEEDR & ~twoBits) | ospeedr;
      port->PUPDR = (port->PUPDR & ~twoBits) | pupdr;
      port->AFR[pad / 8U] =
	(port->AFR[pad / 8U] & ~fourBits) | alternate;
      port->MODER = (port->MODER & ~twoBits) | moder;
    }

  private:
    GPIO_TypeDef * const port;
    const uint32_t pad;
    const uint32_t oneBit;
    const uint32_t twoBits;
    const uint32_t fourBits;
    const uint32_t moder;
    const uint32_t otyper;
    const uint32_t ospeedr;
    const uint32_t pupdr;
    const uint32_t alternate;
    const uint32_t output;
  };

  /// Save/restore only the two pads temporarily changed during recovery.
  class gpio_config_t {
  public:
    gpio_config_t()
      : sda(reinterpret_cast<GPIO_TypeDef *>(PAL_PORT(LINE_I2C_SDA)),
	    PAL_PAD(LINE_I2C_SDA)),
	scl(reinterpret_cast<GPIO_TypeDef *>(PAL_PORT(LINE_I2C_SCL)),
	    PAL_PAD(LINE_I2C_SCL))
    {
    }

    void restore() const
    {
      chSysLock();
      sda.restore();
      scl.restore();
      chSysUnlock();
    }

  private:
    const gpio_pad_config_t sda;
    const gpio_pad_config_t scl;
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
    // I2C lines must never be driven high: open-drain avoids contention if a
    // slave is clock-stretching or still holding the failed bus low.
    palSetLineMode(sclLine, PAL_MODE_OUTPUT_OPENDRAIN);
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

    if (sdaReleased) {
      // Generate a STOP (SDA low -> high while SCL is released high) before
      // restoring the alternate-function configuration.
      palSetLineMode(sdaLine, PAL_MODE_OUTPUT_OPENDRAIN);
      palClearLine(sdaLine);
      palSetLine(sclLine);
      chSysPolledDelayX(US2RTC(STM32_SYSCLK, 10));
      palSetLine(sdaLine);
      chSysPolledDelayX(US2RTC(STM32_SYSCLK, 10));
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

  /** @brief Perform recovery; caller must prevent concurrent I2C transfers. */
  void resetPeripheral()
  {
    const auto config = ExternalI2CD.config;
    i2cStop(&ExternalI2CD);
    if (i2cUnhangBus() == false) {
      DebugTrace("unhang bus I2C1 failed");
    }

    i2cStart(&ExternalI2CD, config);
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
    i2cAcquireBus(&ExternalI2CD);
    resetPeripheral();
    i2cReleaseBus(&ExternalI2CD);
  }

  /** @brief Recover while the caller retains the shared-bus mutex. */
  void resetLocked()
  {
    resetPeripheral();
  }
};
