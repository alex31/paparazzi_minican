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

#ifdef R1M8M
constexpr  UAVCAN::RegTimings timings =  UAVCAN::getTimings(STM32_SYSCLK / 2U,
					  1000, 0.5,
					  8000, 0.75,
					  true);
#elifdef R500K5M
constexpr  UAVCAN::RegTimings timings =  UAVCAN::getTimings(STM32_SYSCLK / 2U,
					  500, 0.5,
					  5000, 0.75,
					  true);
#elifdef R125K250K
constexpr  UAVCAN::RegTimings timings =  UAVCAN::getTimings(STM32_SYSCLK / 2U,
					  125, 0.5,
					  250, 0.66,
					  false);
#elifdef R500K1M
constexpr  UAVCAN::RegTimings timings =  UAVCAN::getTimings(STM32_SYSCLK / 2U,
					  500, 0.5,
					  1000, 0.66,
					  false);
#else
    #error FDRATE should be provided in MAKEFILE
#endif


namespace {
  /*
   * see https://phryniszak.github.io/stm32g-fdcan/
   */

  static const CANConfig cancfg = {
    .op_mode = OPMODE_FDCAN,
    .NBTP = timings.nbtp,
    .DBTP = timings.dbtp,
    .TDCR = timings.tdcr,
    .CCCR = FDCAN_CONFIG_CCCR_BRSE, // Bit-Rate Switch Enable
    .TEST = 0,
    .RXGFC = FDCAN_CONFIG_RXGFC_RRFS // disable hardware filtering : filter table too small
  };

  UAVCAN::Node *slaveNode = nullptr;
  uavcan_protocol_file_ReadRequest readReq;

  
  void processNodeStatus(CanardRxTransfer *transfer,
			 const uavcan_protocol_NodeStatus &nodeStatus)
  {
    static bool done = false;

    if (not done) {
      DebugTrace("processNodeStatus CB from source %u with id %u and timestamp %lu",
		 transfer->source_node_id,
		 transfer->data_type_id,
		 nodeStatus.uptime_sec);
      done = true;
    }
  }

  void processNodeInfoRequest(CanardRxTransfer *transfer,
			      const uavcan_protocol_GetNodeInfoRequest &)
  {
    DebugTrace("request node info CB source %u with id %u\n",
	       transfer->source_node_id,
	       transfer->data_type_id
	       );
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
      uavcan_protocol_NodeStatus nodeStatus;
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
  DeviceStatus start(uint8_t _nodeId)
  {
    static uint8_t nodeId = _nodeId;
    static  UAVCAN::Config uavCanCfg = {
      .cand = CAND2,
      .cancfg = cancfg,
      .busNodeType = UAVCAN::BUS_FD_ONLY,
      .nodeId = nodeId,
      .nodeInfo = {
	.status = {},
	.software_version = {
	  .major = 0,
	  .minor = 1,
	  .optional_field_flags = 0,
	  .vcs_commit = 0,
	  .image_crc = 0
	},
	.hardware_version = {
	  .major = 2,
	  .minor = 0,
	  .unique_id = {},
	  .certificate_of_authenticity = {}
	},
	.name = {10, "minican_V4"}
      },

      .flagCb = [] -> uint8_t {return nodeId;},
      .errorCb = [](const etl::string_view sv) {
	static uint32_t count = 4U;
	if (count) {
	  DebugTrace("UAVDbg : %s", sv.data());
	  --count;
	}
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
    return DeviceStatus(DeviceStatus::ALL, DeviceStatus::OK);
  }
}


namespace {
}
