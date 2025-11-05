#include <ch.h>
#include "etl/vector.h"
#include "UAVCanSlave.hpp"
#include "deviceRessource.hpp"
#include "UAVCAN/persistantParam.hpp"
#include "UAVCAN/pubSub.hpp"
#include "servoRole.hpp"
#include "healthSurvey.hpp"
#include "baro_MPL3115A2_Role.hpp"
#include "escDshotRole.hpp"
#include "sbusTunnelRole.hpp"
#include "telemetryTunnelRole.hpp"
#include "firmwareUpdate.hpp"


#define STR_(x) #x
#define STR(x)  STR_(x)
#define NAME_STRING(prefix, sep, version) prefix sep version
#define DEVICE_NAME NAME_STRING(STR(PLATFORM), "_V", STR(HW_VERSION))


// filter G4 EID 0x0200 0000  MASK 0xFFFF FFFC
// static const FDCANExtendedFilter extFilterList[] = {
//   {
//     .EFID1 = 0x0200'0000,
//     .EFEC =  FILTERING_FEC_FIFO_0,
//     .EFID2 = 0x1FFF'FFFC,
//     ._R1 = 0,
//     .EFT = FILTERING_FT_MASK
//   },
//   {
//     .EFID1 = 0x0300'0000, // F7
//     .EFEC =  FILTERING_FEC_FIFO_1,
//     .EFID2 = 0x1FFF'FFFC,
//     ._R1 = 0,
//     .EFT = FILTERING_FT_MASK
//   }
// };

#ifndef CAN_BITRATE
    #error CAN_BITRATE should be provided in MAKEFILE
#endif



constexpr  UAVCAN::RegTimings timings = UAVCAN::getTimings(STM32_SYSCLK / 2U, CAN_BITRATE);
/*
constexpr  UAVCAN::RegTimings timings =  UAVCAN::getTimings(STM32_SYSCLK / 2U,
					  125, 0.5,
					  250, 0.66,
					  false);
*/

namespace {
  /*
   * see https://phryniszak.github.io/stm32g-fdcan/
   */

  static const CANConfig cancfg = {
    .op_mode = timings.op_mode,
    .NBTP = timings.nbtp,
    .DBTP = timings.dbtp,
    .TDCR = timings.tdcr,
    .CCCR = timings.cccr, // Bit-Rate Switch Enable
    .TEST = 0,
    .RXGFC = FDCAN_CONFIG_RXGFC_RRFS // disable hardware filtering : filter table too small
  };

  UAVCAN::Node *slaveNode = nullptr;
  uavcan_protocol_file_ReadRequest readReq;

  
  void processNodeStatus(CanardRxTransfer *transfer,
			 const uavcan_protocol_NodeStatus &nodeStatus)
  {
#ifdef TRACE
    static bool done = false;

    if (not done) {
      DebugTrace("processNodeStatus CB from source %u with id %u and timestamp %lu",
		 transfer->source_node_id,
		 transfer->data_type_id,
		 nodeStatus.uptime_sec);
      done = true;
    }
#else
    (void) transfer;
    (void) nodeStatus;
#endif
  }

  void processNodeInfoRequest(CanardRxTransfer *transfer,
			      const uavcan_protocol_GetNodeInfoRequest &)
  {
#ifdef TRACE
    DebugTrace("request node info CB source %u with id %u\n",
	       transfer->source_node_id,
	       transfer->data_type_id
	       );
#else
    (void) transfer;
#endif
  }

  void processFirmwareUpdateRequest(CanardRxTransfer *transfer,
			     const uavcan_protocol_file_BeginFirmwareUpdateRequest &msg)
  {
    uavcan_protocol_file_BeginFirmwareUpdateResponse resp = {};
    DebugTrace("processFirmwareUpdate from source %u with id %u\n",
	       transfer->source_node_id,
	       transfer->data_type_id
	       );


    bool ok = FirmwareUpdater::start(slaveNode, msg.image_file_remote_path, resp);
    slaveNode->sendResponse(resp, transfer);
    if (ok) {
      readReq = {
	.offset = 0,
	.path = msg.image_file_remote_path,
      };
      uavcan_protocol_NodeStatus nodeStatus = {};
      nodeStatus.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_SOFTWARE_UPDATE;
      slaveNode->setStatus(nodeStatus);
      slaveNode->sendRequest(readReq, CANARD_TRANSFER_PRIORITY_MEDIUM, transfer->source_node_id);
    }
  }

  void processFileReadResponse(CanardRxTransfer *transfer,
			       const uavcan_protocol_file_ReadResponse &firmwareChunk)
  {
    FirmwareUpdater::newChunk(transfer, firmwareChunk, readReq);
  }

  void processNodeInfoResponse(CanardRxTransfer *transfer,
			       const uavcan_protocol_GetNodeInfoResponse &nodeInfoResp)
  {
#ifdef TRACE
    DebugTrace("response node info CB source %u with id %u\n"
	       "name = %s uptime = %lu  health = %u hw = %u.%u sw = %u.%u",
	       transfer->source_node_id,
	       transfer->data_type_id,
	       nodeInfoResp.name.data,
	       nodeInfoResp.status.uptime_sec,
	       nodeInfoResp.status.health,
	       nodeInfoResp.hardware_version.major,
	       nodeInfoResp.hardware_version.minor,
	       nodeInfoResp.software_version.major,
	       nodeInfoResp.software_version.minor
	       );
#else
    (void) transfer;
    (void) nodeInfoResp;
#endif
  }


  void processGetSetRequest(CanardRxTransfer *transfer,
		     const uavcan_protocol_param_GetSetRequest &req)
  {
    uavcan_protocol_param_GetSetResponse resp;
    const auto& [index, storeVal] = getSetResponse(req, resp);
    if (index > 0) {
      const auto& p = Persistant::Parameter::find(index);
      if (Persistant::Parameter::set(p, storeVal)) {
	// do we store on getset ?
	// it seems that the protocol leave the storage to the
	// processExecuteOpcodeRequest command
	// Ressource::storage.store(index);
      }
    }
    slaveNode->sendResponse(resp, transfer);
  }

  void processRestartNodeRequest(CanardRxTransfer *transfer,
		     const uavcan_protocol_RestartNodeRequest &req)
  {
    uavcan_protocol_RestartNodeResponse resp = {
      .ok = true
    };
    if (req.magic_number != UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_MAGIC_NUMBER) {
      resp.ok = false;
      slaveNode->sendResponse(resp, transfer);
      return;
    }
    slaveNode->sendResponse(resp, transfer);
    chThdSleepMilliseconds(100);
    systemReset();
  }

  void processExecuteOpcodeRequest(CanardRxTransfer *transfer,
		     const uavcan_protocol_param_ExecuteOpcodeRequest &req)
  {
    uavcan_protocol_param_ExecuteOpcodeResponse resp = {
      .argument = req.argument,
      .ok = true
    };
    switch (req.argument) {
    case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE:
      Ressource::storage.storeAll();
      break;
    case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE:
      Ressource::storage.eraseAll();
      break;
    default:
      resp.ok = false;
    }
    slaveNode->sendResponse(resp, transfer);
  }

  etl::vector<RoleBase *, 16> roles;

  template <size_t N>
  struct FixedString {
    char value[N];
    constexpr FixedString(const char (&str)[N]) {
      for (size_t i = 0; i < N; ++i) value[i] = str[i];
    }
    constexpr const char* data() const { return value; }
    constexpr size_t size() const { return N; }
  };


  // add an object of type RoleClass if any of the Params is found
  // in the frozen parameter list
  template <typename RoleClass, FixedString... roleNames>
  void addRole()
  {
    // Vérifie que tous les roleNames existent à la compilation
    static_assert(((Persistant::Parameter::findIndex(roleNames.value) >= 0) && ...),
		  "One of the roleNames was not found!");
    
    // Activation si au moins un est activé à l’exécution
    const bool activateRole =
      ((Persistant::Parameter::get<bool>(Persistant::Parameter::findIndex(roleNames.value))) ||
       ...);
    
    if (not roles.full()) {
      if (activateRole) {
	roles.push_back(new RoleClass);
      }
    } else {
      chSysHalt("roles is full");
    }
  }
  

}

 
namespace CANSlave {
  uint8_t	getNodeId()
  {
    return slaveNode->getNodeId();
  }
  
  DeviceStatus start(int8_t _nodeId)
  {
    static int8_t nodeId = _nodeId;
    static  UAVCAN::Config uavCanCfg = {
      .cand = CAND2,
      .cancfg = cancfg,
      .nodeId = nodeId,
      .nodeInfo = {
	.status = {},
	.software_version = {
	  .major = SW_VERSION_MAJOR,
	  .minor = SW_VERSION_MINOR,
	  .optional_field_flags = 0,
	  .vcs_commit = VCS_COMMIT,
	  .image_crc = 0
	},
	.hardware_version = {
	  .major = HW_VERSION,
	  .minor = 0,
	  .unique_id = {},
	  .certificate_of_authenticity = {}
	},
	.name = {10, DEVICE_NAME}
      },

      .flagCb = [] -> uint8_t {return nodeId;},
      .infoCb = [](const etl::string_view sv) {
#ifdef TRACE
	static uint32_t count = 4U;
	if (count) {
	  DebugTrace("UAVDbg : %s", sv.data());
	  --count;
	}
#else
	(void) sv;
#endif
      }
    };
  
    static UAVCAN::Node node(uavCanCfg);
    
    slaveNode = &node;
    chDbgAssert(STM32_FDCANCLK == 85'000'000, "bad STM32_FDCANCLK source frequency");
    
    // terminal CAN 120 Ohms resistor activation (or not)
    const bool activateR =  PARAM_CGET("can.terminal_resistor");
    palWriteLine(LINE_CAN_TERMR_EN, activateR ? PAL_HIGH : PAL_LOW);

    
    node.subscribeBroadcastMessages<processNodeStatus>();
    node.subscribeResponseMessages<processNodeInfoResponse,
				   processFileReadResponse>();
    node.subscribeRequestMessages<processNodeInfoRequest, processGetSetRequest,
				  processRestartNodeRequest, processExecuteOpcodeRequest,
				  processFirmwareUpdateRequest>();

    addRole<ServoRole, FixedString("role.servo.pwm"),  FixedString("role.servo.smart")>();
    addRole<Baro_MPL3115A2_Role, FixedString("role.i2c.barometer.mpl3115a2")>();
    addRole<EscDshot, FixedString("role.esc.dshot")>();
    addRole<SbusTunnel, FixedString("role.tunnel.sbus")>();
    addRole<TelemetryTunnel, FixedString("role.tunnel.telemetry")>();

    for (auto rp : roles)
      if (const DeviceStatus roleStatus = rp->subscribe(node); not roleStatus)
	return roleStatus;
    
    
    node.start();
    
    for (auto rp : roles)
      if (const DeviceStatus roleStatus = rp->start(node); not roleStatus)
	return roleStatus;
    
    
    HealthSurvey::start(node);
    DebugTrace("Dynamic node_id is %d", node.getNodeId());
    return DeviceStatus(DeviceStatus::ALL, DeviceStatus::OK);
  }
}


namespace {
}
