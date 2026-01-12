/**
 * @file firmwareUpdate.hpp
 * @author Alex Bustico
 * @brief Handles the UAVCAN firmware update process.
 * @details This namespace provides the functions necessary to receive a new firmware
 * image over UAVCAN and store it in the external EEPROM. It manages the
 * uavcan::protocol::file::BeginFirmwareUpdate and uavcan::protocol::file::Read
 * services to download the firmware chunk by chunk.
 */
#pragma once

#include <ch.h>
#include <hal.h>
#include "UAVCAN/pubSub.hpp"
#include "roleBase.hpp"
#include "roleStatus.hpp"
#include "etl/vector.h"
#include "firmwareHeader.hpp"



namespace FirmwareUpdater {
  /**
   * @brief Starts the firmware update process.
   * @details This function is called when a uavcan::protocol::file::BeginFirmwareUpdate
   * request is received.
   * @param node Pointer to the UAVCAN node.
   * @param path The path of the firmware file to be downloaded.
   * @param resp The response to be sent back to the file server.
   * @return `true` if the update process can start, `false` otherwise.
   */
  bool start(UAVCAN::Node *node, const uavcan_protocol_file_Path &path,
	     uavcan_protocol_file_BeginFirmwareUpdateResponse& resp);


  void firstRequest(const uavcan_protocol_file_ReadRequest &firtReq, uint8_t source_node_id);
  
/**
   * @brief Processes a chunk of the firmware data.
   * @details This function is called for each uavcan::protocol::file::Read response
   * received from the file server. It writes the received data chunk into a sector
   * buffer. When the buffer is full, it writes the data to the external EEPROM.
   * When the last chunk is received, it finalizes the update by writing the
   * complete firmware header.
   * @details  Firsts bytes are a header thet permit to validate the firmware version,
   * hardware compatibility and if the update can proceed,
   * it allocates a buffer for the incoming data.
   * @param transfer The CANard transfer metadata.
   * @param firmwareChunk The received firmware data chunk.
   * @param readReq The next file read request to be sent.
   * @return `true` if the chunk was processed successfully, `false` on error.
   */
  bool newChunk(CanardRxTransfer *transfer,
		const uavcan_protocol_file_ReadResponse &firmwareChunk,
		uavcan_protocol_file_ReadRequest &readReq);

  /**
   * @brief Defines the type for the sector buffer used to temporarily store firmware data.
   */
  using SectorBuffer_t = etl::vector<uint8_t, 512>;
  
 
};
