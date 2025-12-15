#include <algorithm>
#include <ch.h>
#include <cstring>
#include "etl/vector.h"
#include "UAVCanSlave.hpp"
#include "deviceRessource.hpp"
#include "UAVCAN/persistantParam.hpp"
#include "UAVCAN/dsdlStringUtils.hpp"
#include "UAVCAN/pubSub.hpp"
#include "nodeParametersEnum.hpp"

#if USE_SERVO_ROLE
#include "servoRole.hpp"
#endif
#include "healthSurvey.hpp"
#if USE_BARO_MPL3115A2_ROLE
#include "baro_MPL3115A2_Role.hpp"
#endif
#if USE_ESC_DSHOT_ROLE
#include "escDshotRole.hpp"
#endif
#if USE_RC_SBUS_ROLE
#include "sbusRole.hpp"
#endif
#if USE_SERIAL_STREAM_ROLE
#include "serialStreamRole.hpp"
#endif
#if USE_LED2812_ROLE
#include "rgbLedRole.hpp"
#endif
#if USE_VOLTMETER_ROLE
#include "voltmeterRole.hpp"
#endif
#if USE_GPS_UBX_ROLE
#include "gpsUbxRole.hpp"
#endif
#if USE_QMC5883_ROLE
#include "qmc5883Role.hpp"
#endif
#include "firmwareUpdate.hpp"
#include "hardwareConf.hpp"
#include "UAVCanHelper.hpp"


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



constexpr  UAVCAN::RegTimings timings = UAVCAN::getTimings(STM32_SYSCLK, CAN_BITRATE);
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

      FirmwareUpdater::firstRequest(readReq, transfer->source_node_id);
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
    bool requestStore = false;
    bool requestReboot = false;
    if (index > 0) {
      const auto& p = Persistant::Parameter::find(index);
      if (Persistant::Parameter::set(p, storeVal)) {
	const int behaviorRaw = static_cast<int>(param_cget<"uavcan.param_set_behavior">());
	const auto behavior = static_cast<ParamSetBehavior>(
	  std::clamp(behaviorRaw, static_cast<int>(SetRam), static_cast<int>(SetRamFlashAndReboot)));

	switch (behavior) {
	case SetRam:
	  break;
	case SetRamFlash:
	  requestStore = true;
	  break;
	case SetRamFlashAndReboot:
	  requestStore = true;
	  requestReboot = true;
	  break;
	default:
	  break;
	}
      }
    }
    slaveNode->sendResponse(resp, transfer);
    if (requestStore) {
      Ressource::storage.store(index);
    }
    if (requestReboot) {
      chThdSleepMilliseconds(100);
      systemReset();
    }
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
  bool addRole()
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
      UAVCAN::Helper::log(*slaveNode, UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR,
			  "UAVCanSlave.cpp::addRole()", "roles vector table too small");
      return false;
    }
    return true;
  }
#ifdef TRACE
  void printOnceInATime(const etl::string_view sv);
#endif
}

 
namespace CANSlave {
  uint8_t	getNodeId()
  {
    return slaveNode->getNodeId();
  }
  
  DeviceStatus start(int8_t _nodeId, bool dynamicId_fd)
  {
    static int8_t nodeId = _nodeId;
    static  UAVCAN::Config uavCanCfg = {
      .cand = CAND2,
      .cancfg = cancfg,
      .nodeId = nodeId,
      .dynamicId_fd = dynamicId_fd,
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
	.name = {sizeof(DEVICE_NAME) - 1U, DEVICE_NAME}
      },

      .flagCb = [] -> uint8_t {return nodeId;},
      .infoCb = [](const etl::string_view sv) {
#ifdef TRACE
	printOnceInATime(sv);
#else
	(void) sv;
#endif
      }
    };
    const auto& nickname = param_cget<"hardware.nickname">();
    UAVCAN::dsdlAppend(uavCanCfg.nodeInfo.name, " ");
    UAVCAN::dsdlAppend(uavCanCfg.nodeInfo.name, nickname.c_str());
    static UAVCAN::Node node(uavCanCfg);
    
    slaveNode = &node;
    
    // terminal CAN 120 Ohms resistor activation (or not)
    const bool activateR =  param_cget<"can.terminal_resistor">();
    palWriteLine(LINE_CAN_TERMR_EN, activateR ? PAL_HIGH : PAL_LOW);

    
    node.subscribeBroadcastMessages<processNodeStatus>();
    node.subscribeResponseMessages<processNodeInfoResponse,
				   processFileReadResponse>();
    node.subscribeRequestMessages<processNodeInfoRequest, processGetSetRequest,
				  processRestartNodeRequest, processExecuteOpcodeRequest,
				  processFirmwareUpdateRequest>();

    bool rolesOk = true;
#if USE_SERVO_ROLE
    rolesOk = rolesOk && addRole<ServoRole, FixedString("ROLE.servo.pwm"),  FixedString("ROLE.servo.smart")>();
#endif
#if USE_BARO_MPL3115A2_ROLE
    rolesOk = rolesOk && addRole<Baro_MPL3115A2_Role, FixedString("ROLE.i2c.barometer.mpl3115a2")>();
#endif
#if USE_QMC5883_ROLE
    rolesOk = rolesOk && addRole<Qmc5883Role, FixedString("ROLE.i2c.magnetometer.q5883")>();
#endif
#if USE_ESC_DSHOT_ROLE
    rolesOk = rolesOk && addRole<EscDshot, FixedString("ROLE.esc.dshot")>();
#endif
#if USE_RC_SBUS_ROLE
    rolesOk = rolesOk && addRole<RC_Sbus, FixedString("ROLE.sbus")>();
#endif
#if USE_GPS_UBX_ROLE
    rolesOk = rolesOk && addRole<GpsUBX, FixedString("ROLE.gnss.ubx")>();
#endif
#if USE_SERIAL_STREAM_ROLE
    rolesOk = rolesOk && addRole<SerialStream, FixedString("ROLE.tunnel.serial")>();
#endif
#if USE_LED2812_ROLE
    rolesOk = rolesOk && addRole<RgbLedRole, FixedString("ROLE.led2812")>();
#endif
#if USE_VOLTMETER_ROLE
    rolesOk = rolesOk && addRole<VoltmeterRole, FixedString("ROLE.voltmeter")>();
#endif

    if (not rolesOk) {
      node.setStatusMode(UAVCAN_PROTOCOL_NODESTATUS_MODE_OFFLINE);
      return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::NB_ROLE_TOO_LARGE);
    }

    for (auto rp : roles)
      if (const DeviceStatus roleStatus = rp->subscribe(node); not roleStatus) {
	return roleStatus;
      }
    
    node.start();
    
    for (auto rp : roles)
      if (const DeviceStatus roleStatus = rp->start(node); not roleStatus) {
	UAVCAN::Helper::log(*slaveNode, UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR,
			    "UAVCanSlave.cpp::start()", roleStatus.describe());
	
	return roleStatus;
      }
    
    
    if (param_cget<"ROLE.health.survey">()) {
      HealthSurvey::start(node);
    }
    
    node.infoCb("Dynamic node_id is %d", node.getNodeId());
    node.setStatusMode(UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL);
    return DeviceStatus(DeviceStatus::ALL, DeviceStatus::OK);
  }
}


namespace {
#ifdef TRACE
 void printOnceInATime(const etl::string_view sv)
 {
   static etl::string<120> lastMsg = "";
   static systime_t lastTime = 0;
   const systime_t now = chVTGetSystemTimeX();
   
   const bool canRedisplay = chTimeDiffX(lastTime, now) > TIME_S2I(15);

   if ((sv != lastMsg) or canRedisplay) {
     lastMsg = sv;
     lastTime = now;
     DebugTrace("UAVDbg : %s", sv.data());
   }
 }
#endif
}
  
