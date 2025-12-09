/**
 * @file firmwareUpdate.cpp
 * @author Alex Bustico
 * @brief Implements the UAVCAN firmware update process.
 */

#include <algorithm>

#include "firmwareUpdate.hpp"
#include "hardwareConf.hpp"
#include "stdutil++.hpp"
#include "eeprom_stm_m95p.hpp"
#include "MFS.hpp"
#include "etl/string_view.h"
#include "etl/span.h"
#include "hardwareConf.hpp"
#include "UAVCAN/dsdlStringUtils.hpp"
#include "UAVCanHelper.hpp"
/*
    TODO :

    * gerer un entète pour verifier la compatibilité
    * il va y avoir deux entêtes : 1 pour la flash que l'on laisse inchangée,
      1 pour la generation de firmware qui est nouveau et imposé par la façon
      dont dronegui envoie le firmware : sans conserver le nom de l'image (donc
      pas de possibilité de faire de check sur le nom de l'image)

      ° sur un message beginFirmwareUpdate :
        + supprimer le check sur le nom
	+ dans la callback newChunck : gerer un etat, et coller les premiers octets
	  qui arrivent dans une variable toolChainHeader
	+ verifier que le magic est bon (le même que pour la flash) (return error unless)
	+ verifier que le nom de la board est bon (return error unless)
	+ stoquer en flash les octets qui ne sont pas le header
	+ lire le reste du firmware et le stoquer en flash
	+ verifier que la taille lue est conforme
	+ une fois certain que les crc32 utilisés sont les mêmes entre le script perl et
	  le firmware : comparer les CRC
	+ comme avant : mettre à jour le header flash pour que le bootloader flashe l'appli
	  au prochain demarrage
      
 */




namespace FirmwareUpdater {
  ///< Buffer to hold a sector of the firmware image during download.
  SectorBuffer_t *sectorBuffer = nullptr; // if not null : transfert in progress
}

namespace  {
  size_t currentFileIndex = 0;
  UAVCAN::Node *slaveNode = nullptr;
  Eeprom_M95::Device *m95p = nullptr;
  systime_t timestamp = 0;
  // it it goes 10 seconds withoud fileread response, current update is canceled
  static constexpr sysinterval_t firmwareUpdateTimout = TIME_S2I(10);
  bool storeSectorBufferInEeprom(bool final = false);
  Firmware::FirmwareHeader_t  IN_DMA_SECTION(firmwareHeader);
  Firmware::ToolchainHeader_t toolChainHeader;
  static_assert(sizeof(firmwareHeader) <= 512);

  bool fwIsCompatible(const Firmware::ToolchainHeader_t& toolChainHeader);
  Firmware::Flash fwValidity(const Firmware::ToolchainHeader_t& hdr);
  virtual_timer_t vtRequest;
  struct {
    uavcan_protocol_file_ReadRequest readReq;
    uint8_t source_node_id;
    thread_t *requestWorker;
    binary_semaphore_t reqSem;
  } currentFileRequest = {};

  void requestWorker(void *);
  void releaseSectorBuffer();
}


/**
 * @brief Starts the firmware update process.
 */
bool FirmwareUpdater::start(UAVCAN::Node *node, const uavcan_protocol_file_Path &_path,
			    uavcan_protocol_file_BeginFirmwareUpdateResponse& resp)
{
  etl::string_view path(reinterpret_cast<const char*>(_path.path.data), _path.path.len);
  slaveNode = node;
  chVTObjectInit(&vtRequest);
  m95p = MFS::getDevice();
  crcInit();
  crcStart(&CRCD1, &Firmware::crcK4Config);
  bool aborted = false;

  m95p->read(firmwareHeader.headerEepromAddr,
	     etl::span(reinterpret_cast<uint8_t*>(&firmwareHeader), sizeof(firmwareHeader)));
  firmwareHeader.size = 0;
  auto doResp = [&resp] (uint8_t err, etl::string_view str) {
    resp.error = err;
    UAVCAN::dsdlAssign(resp.optional_error_message, str);
  };


  if (sectorBuffer != nullptr)  {
    if (chTimeDiffX(timestamp, chVTGetSystemTimeX()) < firmwareUpdateTimout) {
      doResp(UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_IN_PROGRESS,
	     "update in progress");
      return false;
    } else {
      releaseSectorBuffer();
      aborted = true;
    }
  }
  
  if (FirmwareUpdater::sectorBuffer = new SectorBuffer_t; FirmwareUpdater::sectorBuffer == nullptr) {
    doResp(UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_UNKNOWN,
	   "internal malloc error");
    return false;
  }
  timestamp = chVTGetSystemTimeX();

  doResp(UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_OK,
	 aborted ? "previous attempt aborted, restart update" :
	 "start update");

  if (currentFileRequest.requestWorker == nullptr) {
    chBSemObjectInit(&currentFileRequest.reqSem, true); // start taken so helper blocks
    currentFileRequest.requestWorker =
      chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(512), "file_chunck_requester",
			  chThdGetPriorityX() + 1,
			  &requestWorker, nullptr);
  }
    
  return true;
}

void FirmwareUpdater::firstRequest(const uavcan_protocol_file_ReadRequest &firtReq,
				   uint8_t source_node_id)
{
  slaveNode->setStatusMode(UAVCAN_PROTOCOL_NODESTATUS_MODE_SOFTWARE_UPDATE);
  currentFileRequest.readReq = firtReq;
  currentFileRequest.source_node_id = source_node_id;
  chBSemSignal(&currentFileRequest.reqSem);
}


/**
 * @brief Processes a received firmware chunk.
 */
bool FirmwareUpdater::newChunk(CanardRxTransfer *transfer,
			       const uavcan_protocol_file_ReadResponse &firmwareChunk,
			       uavcan_protocol_file_ReadRequest &readReq)
{
  static constexpr size_t hdrLen =  sizeof(toolChainHeader);
  chVTReset(&vtRequest);

  // Ignore late responses if the update has already been aborted.
  if (!FirmwareUpdater::sectorBuffer) {
    return false;
  }
  
  if (firmwareChunk.error.value != UAVCAN_PROTOCOL_FILE_ERROR_OK) {
    DebugTrace("DBG> firmwareChunk.error.value != UAVCAN_PROTOCOL_FILE_ERROR_OK : %u",
	       firmwareChunk.error.value);
    releaseSectorBuffer();
    return false;
  }
  
  timestamp = chVTGetSystemTimeX();
  if (firmwareChunk.data.len == 0) { // End of file
    storeSectorBufferInEeprom(true);
    releaseSectorBuffer();
    // Restart : the bootloader will flash from spi eeprom to mcu flash
    systemReset();
  } else {
    if (currentFileIndex == 0) {
      // first chunk : it begin with the toochainHeader
      // we don't manage case where chunck is smaller than toochainHeader, in practice
      // it should never occurs
      if (firmwareChunk.data.len < hdrLen) {
	slaveNode->infoCb("invalid first uavcan_protocol_file_ReadResponse frame size < 48");
	return false;
      }
      // first 48 bytes chunk is toolChainHeader
      toolChainHeader = firmwareChunk;
      if (not fwIsCompatible(toolChainHeader)) {
	slaveNode->infoCb("ERROR: firmware is not for this platform");
	slaveNode->setStatusMode(UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL);
	UAVCAN::Helper::log(*slaveNode, UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_WARNING,
			    "firmware updater",
			    "invalid firmware image");
	return false;
      } else {
	slaveNode->infoCb("COMPATIBLE firmware");
      }
      // the remaining bytes are the actual firmware
      sectorBuffer->insert(sectorBuffer->begin(), firmwareChunk.data.data + hdrLen,
			   firmwareChunk.data.data + firmwareChunk.data.len);
    } else {
      // following chunks
      const size_t transferSize = std::min(sectorBuffer->available(),
					   static_cast<size_t>(firmwareChunk.data.len));
      sectorBuffer->insert(sectorBuffer->end(), firmwareChunk.data.data,
			   firmwareChunk.data.data + transferSize);
      
      if (sectorBuffer->full()) {
	storeSectorBufferInEeprom();
	sectorBuffer->assign(firmwareChunk.data.data + transferSize,
			     firmwareChunk.data.data + firmwareChunk.data.len);
      }
    }
    currentFileIndex += firmwareChunk.data.len;
    // Request next chunk
    readReq.offset = currentFileIndex;
    currentFileRequest.readReq = readReq;
    currentFileRequest.source_node_id = transfer->source_node_id;
    // delegate the request sending to a helper thread that also send retries
    // after timeout
    chBSemSignal(&currentFileRequest.reqSem);
  }
  
  return true;
}

extern uint32_t application_start;
namespace {
  /**
   * @brief Writes the buffered firmware data to the EEPROM.
   */
  bool storeSectorBufferInEeprom(bool final) {
    if (not FirmwareUpdater::sectorBuffer->empty()) {
      crcCalc(&CRCD1, FirmwareUpdater::sectorBuffer->data(),
       	      FirmwareUpdater::sectorBuffer->size());
      m95p->write(firmwareHeader.bank1EepromAddr + firmwareHeader.size,
      		  *FirmwareUpdater::sectorBuffer);
      firmwareHeader.size += FirmwareUpdater::sectorBuffer->size();
    }
    if (final) {
      if (firmwareHeader.magicNumber != Firmware::magicNumberCheck) {
	firmwareHeader.magicNumber = Firmware::magicNumberCheck;
	firmwareHeader.eepromCycleCount = 1;
      } 
      firmwareHeader.versionProtocol = firmwareHeader.versionProtocolCheck;
      firmwareHeader.flashAddress = (uint32_t) &application_start; // get from .ld file
      firmwareHeader.crc32k4 = crcGetFinalValue(&CRCD1);
      firmwareHeader.state = fwValidity(toolChainHeader);
      firmwareHeader.headerLen = sizeof firmwareHeader;
      m95p->write(firmwareHeader.headerEepromAddr,
		  etl::span(reinterpret_cast<const uint8_t*>(&firmwareHeader), sizeof(firmwareHeader)));
      crcReset(&CRCD1);
      m95p->waitReady(); // ensure that header is written to flash before continuing
    }
    return true;
  }


  bool fwIsCompatible(const Firmware::ToolchainHeader_t& hdr)
  {
    if (hdr.magicNumber != Firmware::magicNumberCheck)
      return false;

    if (hdr.hwVerMajor != HW_VERSION)
      return false;

    if (strcmp(hdr.platform, DEVICE_NAME) != 0)
      return false;
    
    return true;
  }


  Firmware::Flash fwValidity(const Firmware::ToolchainHeader_t& hdr)
  {
    if (const size_t binarySize = currentFileIndex - sizeof toolChainHeader;
	hdr.fwSize != binarySize) {
      slaveNode->infoCb("hdr.fwSize (%lu) != binarySize (%u)", hdr.fwSize, binarySize);
      return Firmware::Flash::LEN_ERROR;
    }

    if (hdr.fwCrc32 != firmwareHeader.crc32k4) {
      slaveNode->infoCb("fwValidity crc fail");
      //      DebugTrace("DBG> hdr.fwCrc32(0x%lx) != firmwareHeader.crc32k4(0x%lx)",
      //	 hdr.fwCrc32,  firmwareHeader.crc32k4);
      return Firmware::Flash::CRC_ERROR;
    }
    
    return Firmware::Flash::REQUIRED;
  }



  void releaseSectorBuffer() {
    chSysLock();
    delete FirmwareUpdater::sectorBuffer;
    FirmwareUpdater::sectorBuffer = nullptr;
    chSysUnlock();
    currentFileIndex = 0;
  }


  void requestWorker(void *) {
    while(true) {
      chBSemWait(&currentFileRequest.reqSem);
      static uint32_t lastOffset = 0xffffffff;

      if (FirmwareUpdater::sectorBuffer) {
	slaveNode->sendRequest(currentFileRequest.readReq, CANARD_TRANSFER_PRIORITY_MEDIUM,
			       currentFileRequest.source_node_id);
#ifdef TRACE
	DebugTrace("DBG> ask %s chunk %llu/%lu",
		   lastOffset == currentFileRequest.readReq.offset ? "** AGAIN **" : "",
		   currentFileRequest.readReq.offset - sizeof(Firmware::ToolchainHeader_t),
		   toolChainHeader.fwSize);
#else
	(void) lastOffset;
#endif
	lastOffset = currentFileRequest.readReq.offset;
	chVTSet(&vtRequest,
		TIME_MS2I(300),
		[](ch_virtual_timer*, void*) {
		  chSysLockFromISR();
		  chBSemSignalI(&currentFileRequest.reqSem);
		  chSysUnlockFromISR();
		},
		nullptr);
      }
    }
  }
}
