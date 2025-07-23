#pragma once

#include <ch.h>
#include <hal.h>
#include "UAVCAN/pubSub.hpp"
#include "roleBase.hpp"
#include "roleStatus.hpp"
#include "etl/vector.h"
#include "etl/string.h"
#include "hal_stm32_crc_v1.h"



namespace FirmwareUpdater {
  bool start(UAVCAN::Node *node, const uavcan_protocol_file_Path &path,
	     uavcan_protocol_file_BeginFirmwareUpdateResponse& resp);
  bool newChunk(CanardRxTransfer *transfer,
		const uavcan_protocol_file_ReadResponse &firmwareChunk,
		uavcan_protocol_file_ReadRequest &readReq);
  using SectorBuffer_t = etl::vector<uint8_t, 512>;
  
  struct FirmwareHeader_t {
    uint32_t magicNumber;
    etl::string<32> version = {};
    uint32_t size = 0;
    uint32_t crc32k4 = 0;
    void* flashAddress = 0x0;
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

};
