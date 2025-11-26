#include <algorithm>

#include "gpsUbxRole.hpp"
#include "hardwareConf.hpp"
#include "ressourceManager.hpp"
#include "stdutil++.hpp"

#if PLATFORM_MICROCAN
#include "dynamicPinConfig.hpp"
#endif

bool GpsUBX::navPvtCb(const UBX::NavPvt& msg)
{
  auto* self = RoleCrtp<GpsUBX>::singleton;
  if (!self || !self->m_node) {
    return false;
  }

  self->satsUsed = msg.numSV;

  uavcan_equipment_gnss_Fix fix{};
  // Timestamp node local (temps de publication)
  fix.timestamp.usec = static_cast<uint64_t>(TIME_I2US(chVTGetSystemTimeX()));
  fix.gnss_timestamp.usec = static_cast<uint64_t>(msg.iTOW) * 1000ULL; // ms -> usec
  fix.gnss_time_standard = UAVCAN_EQUIPMENT_GNSS_FIX_GNSS_TIME_STANDARD_GPS;
  fix.num_leap_seconds = UAVCAN_EQUIPMENT_GNSS_FIX_NUM_LEAP_SECONDS_UNKNOWN;

  fix.longitude_deg_1e8 = static_cast<int64_t>(msg.lon) * 10; // 1e-7 -> 1e-8
  fix.latitude_deg_1e8  = static_cast<int64_t>(msg.lat) * 10;
  fix.height_ellipsoid_mm = msg.height;
  fix.height_msl_mm = msg.hMSL;

  fix.ned_velocity[0] = static_cast<float>(msg.velN) / 1000.F;
  fix.ned_velocity[1] = static_cast<float>(msg.velE) / 1000.F;
  fix.ned_velocity[2] = static_cast<float>(msg.velD) / 1000.F;

  fix.sats_used = msg.numSV;
  switch (msg.fixType) {
  case 0: fix.status = UAVCAN_EQUIPMENT_GNSS_FIX_STATUS_NO_FIX; break;
  case 1: fix.status = UAVCAN_EQUIPMENT_GNSS_FIX_STATUS_TIME_ONLY; break;
  case 2: fix.status = UAVCAN_EQUIPMENT_GNSS_FIX_STATUS_2D_FIX; break;
  default: fix.status = UAVCAN_EQUIPMENT_GNSS_FIX_STATUS_3D_FIX; break;
  }

  fix.pdop = static_cast<float>(msg.pDOP) * 0.01F;
  fix.position_covariance.len = 0;
  fix.velocity_covariance.len = 0;

  return self->m_node->sendBroadcast(fix) == UAVCAN::Node::CAN_OK;
}

bool GpsUBX::navDopCb(const UBX::NavDop& msg)
{
  auto* self = RoleCrtp<GpsUBX>::singleton;
  if (!self || !self->m_node) {
    return false;
  }

  self->dopCache = {
    .gdop = static_cast<float>(msg.gDOP) * 0.01F,
    .pdop = static_cast<float>(msg.pDOP) * 0.01F,
    .tdop = static_cast<float>(msg.tDOP) * 0.01F,
    .vdop = static_cast<float>(msg.vDOP) * 0.01F,
    .hdop = static_cast<float>(msg.hDOP) * 0.01F,
    .ndop = static_cast<float>(msg.nDOP) * 0.01F,
    .edop = static_cast<float>(msg.eDOP) * 0.01F,
    .valid = true
  };

  uavcan_equipment_gnss_Auxiliary aux{};
  aux.gdop = self->dopCache.gdop;
  aux.pdop = self->dopCache.pdop;
  aux.hdop = self->dopCache.hdop;
  aux.vdop = self->dopCache.vdop;
  aux.tdop = self->dopCache.tdop;
  aux.ndop = self->dopCache.ndop;
  aux.edop = self->dopCache.edop;
  aux.sats_visible = self->satsVisible;
  aux.sats_used = self->satsUsed;

  return self->m_node->sendBroadcast(aux) == UAVCAN::Node::CAN_OK;
}

bool GpsUBX::navSatCb(const UBX::NavSat& msg)
{
  auto* self = RoleCrtp<GpsUBX>::singleton;
  if (!self || !self->m_node) {
    return false;
  }

  self->satsVisible = msg.numSvs;

  if (!self->dopCache.valid) {
    return true; // rien à publier tant qu'on n'a pas de DOP
  }

  uavcan_equipment_gnss_Auxiliary aux{};
  aux.gdop = self->dopCache.gdop;
  aux.pdop = self->dopCache.pdop;
  aux.hdop = self->dopCache.hdop;
  aux.vdop = self->dopCache.vdop;
  aux.tdop = self->dopCache.tdop;
  aux.ndop = self->dopCache.ndop;
  aux.edop = self->dopCache.edop;
  aux.sats_visible = self->satsVisible;
  aux.sats_used = self->satsUsed;

  return self->m_node->sendBroadcast(aux) == UAVCAN::Node::CAN_OK;
}

GpsUBX::GpsUBX()
  : decoder(config)
{
}



/*
  dans le subscribe : rien à faire : on ne reçoit pas de messages UAVCan

  dans le start :
  + demarrer la liaison série avec les bon paramètres
  + creer un thread qui :
   ° écoute sur la liaison série avec un buffer de 600 et un timout sur 1 ou 2 caractères
   ° nourrit un objet gpsUbx
   ° dans la CB de gpsUbx : si le message est complet/correct :
     construire un message  et l'envoyer via UAVCAN
 */

namespace {
 UARTConfig gpscfg =
  {
    .txend1_cb =nullptr,
    .txend2_cb = nullptr,
    .rxend_cb = nullptr,
    .rxchar_cb = nullptr,
    .rxerr_cb = nullptr,
    // .timeout_cb = [](hal_uart_driver *) {
    //   palSetLine(LINE_DBG_TIMOUT_CB);
    //   chSysPolledDelayX(1);
    //   palClearLine(LINE_DBG_TIMOUT_CB);
    // },
    .timeout_cb = nullptr,
    .timeout = 6,
    .speed = 0, // will be be at init
    .cr1 = USART_CR1_RTOIE, 
    .cr2 = USART_CR2_STOP1_BITS | USART_CR2_RTOEN,
    .cr3 = 0
  };



  
}


// It is assumed that thread live forever (until poweroff) dynamic memory allocation is
// done there to not use ram if the role is not enabled. It's normal that the memory is never
// released

DeviceStatus GpsUBX::subscribe(UAVCAN::Node&)
{
  return DeviceStatus(DeviceStatus::GPS_ROLE);
}


DeviceStatus GpsUBX::start(UAVCAN::Node& node)
{
  using HR = HWResource;
  DeviceStatus status(DeviceStatus::GPS_ROLE);
  
  m_node = &node;
  
  gpscfg.speed =  PARAM_CGET("role.gnss.ubx.baudrate");
  uartStart(&ExternalUARTD, &gpscfg);
  
  // use serial rx
#if PLATFORM_MINICAN
  if (not boardResource.tryAcquire(HR::USART_2, HR::PB04)) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT);
  }
#endif
  
#if PLATFORM_MICROCAN
  if (not boardResource.tryAcquire(HR::USART_1, HR::PA10, HR::F3)) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT);
  }
  //  MSB F4 F3 F2a F0b F0a LSB
  DynPin::setScenario(DynPin::Scenario::UART, 0b01000);
#endif
  
  chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(1024), "gps", NORMALPRIO, 
		      &Trampoline<&GpsUBX::periodic>::fn, this);
  
  return status;
}

void GpsUBX::periodic(void *)
{
  size_t size;
  while (true) {
    size = sizeof(frame);
    uartReceiveTimeout(&ExternalUARTD, &size, frame, TIME_INFINITE);
    if (size) {
      decoder.feed({frame, size});
    }
  }
}
