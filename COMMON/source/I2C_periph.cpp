#include <ch.h>
#include <hal.h>

#include "I2C_periph.hpp"
#include "hardwareConf.hpp"
#include "ressourceManager.hpp"
#include "dynamicPinConfig.hpp"
#include "UAVCAN/persistantParam.hpp"
#include "stdutil.h"

namespace {
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

  bool started = false;
}


namespace I2CPeriph
{
  DeviceStatus start()
  {
    // already started : nothing to do
    if (started)
      return DeviceStatus(DeviceStatus::I2C);

    using HR = HWResource;
    
#if PLATFORM_MINICAN
    if (not boardResource.tryAcquire(HR::PA15, HR::PB07, HR::I2C_1)) {
      return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT,
			  std::to_underlying(HR::I2C_1));
    }
#elif PLATFORM_MICROCAN
    if (not boardResource.tryAcquire(HR::I2C_2, HR::F1, HR::F2)) {
      return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT,
			  std::to_underlying(HR::I2C_2));
    }
    DynPin::setScenario(DynPin::Scenario::I2C);
#endif
    DynPin::i2cActivatePullup();
    
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
  
  void reset()
  {
    const auto config = ExternalI2CD.config;
    i2cStop(&ExternalI2CD);
    if (DynPin::i2cUnhangBus(&ExternalI2CD) == false) {
      DebugTrace("unhang bus I2C1 failed");
    }
    
    i2cStart(&ExternalI2CD, config);
  }
};







