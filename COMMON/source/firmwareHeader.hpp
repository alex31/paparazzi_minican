#pragma once

#include "etl/string.h"
#include "hal_stm32_crc_v1.h"

namespace Firmware {
  struct FirmwareHeader_t {
    uint32_t magicNumber;
    etl::string<32> version = {};
    uint32_t size = 0;
    uint32_t crc32k4 = 0;
    uint32_t flashAddress = 0x0;
    bool     flashToMCU = false;
    bool     bootloaderFlashSuccess = false;
    uint8_t  bankInUse = 0;
    uint16_t headerLen = 0;
    static constexpr uint32_t magicNumberCheck = 0xF0CACC1A;
    static constexpr uint32_t headerEepromAddr = 1024*1024;
    static constexpr uint32_t bank1EepromAddr = headerEepromAddr + 512;
    static constexpr uint32_t bank2EepromAddr = bank1EepromAddr + (512 * 1024);
  };
  
  static constexpr CRCConfig crcK4Config = {
    .poly_size = 32,
    .poly = 0xC9D204F5,
    .initial_val = 0xFFFFFFFF,
    .final_val = 0xFFFFFFFF,
    .reflect_data = true,
    .reflect_remainder = true
  };
}
