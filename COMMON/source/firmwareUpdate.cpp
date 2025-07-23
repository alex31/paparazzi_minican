#include <algorithm>

#include "firmwareUpdate.hpp"
#include "hardwareConf.hpp"
#include "stdutil++.hpp"
#include "eeprom_stm_m95p.hpp"
#include "MFS.hpp"
#include "etl/string_view.h"

/*
  TODO :
 */

namespace FirmwareUpdater {
  SectorBuffer_t *sectorBuffer = nullptr; // if not null : transfert in progress
}

namespace  {
  size_t currentFileIndex = 0;
  UAVCAN::Node *slaveNode = nullptr;
  Eeprom_M95::Device *m95p = nullptr;

  bool storeSectorBufferInEeprom(bool final = false);
  const CRCConfig crcConfig = {
    .poly_size = 32,
    .poly = 0xC9D204F5,
    .initial_val = 0xFFFFFFFF,
    .final_val = 0xFFFFFFFF,
    .reflect_data = true,
    .reflect_remainder = true
  };
  FirmwareUpdater::FirmwareHeader_t firmwareHeader;
  static_assert(sizeof(firmwareHeader) <= 512);
}


bool FirmwareUpdater::start(UAVCAN::Node *node, const uavcan_protocol_file_Path &_path,
			    uavcan_protocol_file_BeginFirmwareUpdateResponse& resp)
{
  etl::string_view path(reinterpret_cast<const char*>(_path.path.data), _path.path.len);
  slaveNode = node;
  bool wrongFileName = false;
  m95p = MFS::getDevice();
  crcInit();
  crcStart(&CRCD1, &crcConfig);
  
  auto doResp = [&resp] (uint8_t err, etl::string_view str) {
    resp.error = err;
    resp.optional_error_message.len = str.size();
    memcpy(resp.optional_error_message.data, str.data(),
	   std::min(sizeof(resp.optional_error_message.data), str.size()));
  };
  /*
    ° if filename is *NOT* suitable for minican -> answer error "incompatible", return
    ° if filename is same as stored in header : answer error "same version", return
  */
  etl::string_view ver_path;
  auto pos = path.find(':');
  if (pos == etl::string_view::npos) {
    wrongFileName = true;
  } else if (path.substr(0, pos) != "minican") {
    wrongFileName = true;
  } else if (ver_path = path.substr(pos + 1); ver_path.empty()) {
    wrongFileName = true;
  }

  if (wrongFileName) {
    doResp(UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_UNKNOWN,
	   "invalid firmware name : need minican:version");
    return false;
  }
  
  if (ver_path == firmwareHeader.version) {
    doResp(UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_UNKNOWN,
	   "firmware version is already flashed or will be flashed at next restart");
    return false;
  }

  // will need to be deallocated after last chunk of firmware is received and written to SPI EEPROM
  if (sectorBuffer != nullptr)  {
    // timout management ?
    doResp(UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_IN_PROGRESS,
	   "update in progress");
    return false;
  }
  
  if (FirmwareUpdater::sectorBuffer = new SectorBuffer_t; FirmwareUpdater::sectorBuffer == nullptr) {
    // timout management ?
    doResp(UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_UNKNOWN,
	   "internal malloc error");
    return false;
  }
  
  return true;
}

bool FirmwareUpdater::newChunk(CanardRxTransfer *transfer,
			       const uavcan_protocol_file_ReadResponse &firmwareChunk,
			       uavcan_protocol_file_ReadRequest &readReq)
{
  /*
    * write chunk to EEPROM, seek currentFileIndex
    * ask for next chunk
    * if chunk length is 0 : transfert is complete
    ¤ delete sectorBuffer
    ¤ sectorBuffer = nullptr : ready for next upgrade
    
  */
  if (firmwareChunk.error.value != UAVCAN_PROTOCOL_FILE_ERROR_OK) {
    delete sectorBuffer;
    sectorBuffer = nullptr;
    return false;
  }
  
  if (firmwareChunk.data.len == 0) {
    // DebugTrace("DBG> end of file");
    storeSectorBufferInEeprom(true);
    delete sectorBuffer;
    sectorBuffer = nullptr;
    currentFileIndex = 0;
  } else {
    const size_t transferSize = std::min(sectorBuffer->available(),
					 static_cast<size_t>(firmwareChunk.data.len));
    sectorBuffer->insert(sectorBuffer->end(), firmwareChunk.data.data,
			 firmwareChunk.data.data + transferSize);
    currentFileIndex += firmwareChunk.data.len;
    // ask for next chunk
    readReq.offset = currentFileIndex;
    // DebugTrace("DBG> ask for offset %u", currentFileIndex);
    if (sectorBuffer->full()) {
      storeSectorBufferInEeprom();
      sectorBuffer->assign(firmwareChunk.data.data + transferSize, firmwareChunk.data.data + firmwareChunk.data.len);
    }
    slaveNode->sendRequest(readReq, CANARD_TRANSFER_PRIORITY_MEDIUM, transfer->source_node_id);
  }
  
  return true;
}

extern uint32_t application_start;
namespace {
  bool storeSectorBufferInEeprom(bool final) {
    if (not FirmwareUpdater::sectorBuffer->empty()) {
      crcCalc(&CRCD1, FirmwareUpdater::sectorBuffer->data(),
       	      FirmwareUpdater::sectorBuffer->size());
      m95p->write(firmwareHeader.bank1EepromAddr + firmwareHeader.size,
      		  *FirmwareUpdater::sectorBuffer);
      firmwareHeader.size += FirmwareUpdater::sectorBuffer->size();
    }
    if (final) {
      firmwareHeader.flashAddress = &application_start; // get from .ld file
      firmwareHeader.crc32k4 = crcGetFinalValue(&CRCD1);
      //      DebugTrace("crc = 0x%lx", firmwareHeader.crc32k4);
      firmwareHeader.flashToMCU = true;
      firmwareHeader.headerLen = sizeof firmwareHeader;
      m95p->write(firmwareHeader.headerEepromAddr,
		  std::span(reinterpret_cast<const uint8_t*>(&firmwareHeader), sizeof(firmwareHeader)));
      crcReset(&CRCD1);
    }
    return true;
  }
}
