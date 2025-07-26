#pragma once

#include <ch.h>
#include <hal.h>
#include "UAVCAN/pubSub.hpp"
#include "roleBase.hpp"
#include "roleStatus.hpp"
#include "etl/vector.h"
#include "firmwareHeader.hpp"



namespace FirmwareUpdater {
  bool start(UAVCAN::Node *node, const uavcan_protocol_file_Path &path,
	     uavcan_protocol_file_BeginFirmwareUpdateResponse& resp);
  bool newChunk(CanardRxTransfer *transfer,
		const uavcan_protocol_file_ReadResponse &firmwareChunk,
		uavcan_protocol_file_ReadRequest &readReq);
  using SectorBuffer_t = etl::vector<uint8_t, 512>;
  
 
};
