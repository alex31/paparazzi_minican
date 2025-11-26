#include <algorithm>
#include <cstdint>

#include "gpsUbxRole.hpp"
#include "hardwareConf.hpp"
#include "ressourceManager.hpp"
#include "stdutil++.hpp"

#if PLATFORM_MICROCAN
#include "dynamicPinConfig.hpp"
#endif

namespace {
  bool utcToUnixUsec(const UBX::NavPvt& msg, uint64_t& out);
}

bool GpsUBX::navPvtCb(const UBX::NavPvt& msg)
{
  auto* self = RoleCrtp<GpsUBX>::singleton;
  if (!self || !self->m_node) {
    return false;
  }

  self->satsUsed = msg.numSV;

  uavcan_equipment_gnss_Fix2 fix{};
  // Timestamp node local (temps de publication)
  fix.timestamp.usec = static_cast<uint64_t>(TIME_I2US(chVTGetSystemTimeX()));
  if (!utcToUnixUsec(msg, fix.gnss_timestamp.usec)) {
    fix.gnss_timestamp.usec = 0;
  }
  fix.gnss_time_standard = UAVCAN_EQUIPMENT_GNSS_FIX2_GNSS_TIME_STANDARD_UTC;
  fix.num_leap_seconds = UAVCAN_EQUIPMENT_GNSS_FIX2_NUM_LEAP_SECONDS_UNKNOWN;

  fix.longitude_deg_1e8 = static_cast<int64_t>(msg.lon) * 10; // 1e-7 -> 1e-8
  fix.latitude_deg_1e8  = static_cast<int64_t>(msg.lat) * 10;
  fix.height_ellipsoid_mm = msg.height;
  fix.height_msl_mm = msg.hMSL;

  fix.ned_velocity[0] = static_cast<float>(msg.velN) / 1000.F;
  fix.ned_velocity[1] = static_cast<float>(msg.velE) / 1000.F;
  fix.ned_velocity[2] = static_cast<float>(msg.velD) / 1000.F;

  fix.sats_used = msg.numSV;
  if (msg.flags.gnssFixOK == 0) {
    fix.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_NO_FIX;
  } else {
    switch (static_cast<UBX::NavFixType>(msg.fixType)) {
    case UBX::NavFixType::NO_FIX:
    case UBX::NavFixType::DEAD_RECKONING_ONLY:
      fix.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_NO_FIX;
      break;
    case UBX::NavFixType::TWO_D:
      fix.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_2D_FIX;
      break;
    case UBX::NavFixType::THREE_D:
    case UBX::NavFixType::GNSS_AND_DR:
      fix.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX;
      break;
    case UBX::NavFixType::TIME_ONLY:
      fix.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_TIME_ONLY;
      break;
    default:
      DebugTrace("UBX PVT fixType inconnu: %u", static_cast<unsigned>(msg.fixType));
      fix.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_NO_FIX;
      break;
    }
  }

  fix.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_SINGLE;
  fix.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_DGPS_OTHER;
  if (msg.flags.carrSoln == 1) {
    fix.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK;
    fix.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FLOAT;
  } else if (msg.flags.carrSoln == 2) {
    fix.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK;
    fix.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FIXED;
  } else if (msg.flags.diffSoln) {
    fix.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_DGPS;
    fix.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_DGPS_OTHER;
  }

  // Covariance diag (len=6) suivant l’usage ArduPilot : Pn, Pe, Pd, Vn, Ve, Vd (variances).
  const float hacc_m = static_cast<float>(msg.hAcc) * 0.001F; // mm -> m
  const float vacc_m = static_cast<float>(msg.vAcc) * 0.001F;
  const float sacc_mps = static_cast<float>(msg.sAcc) * 0.001F; // mm/s -> m/s
  const float hacc_var = hacc_m * hacc_m;
  const float vacc_var = vacc_m * vacc_m;
  const float sacc_var = sacc_mps * sacc_mps;
  fix.covariance.len = 6;
  fix.covariance.data[0] = hacc_var; // Pn
  fix.covariance.data[1] = hacc_var; // Pe
  fix.covariance.data[2] = vacc_var; // Pd
  fix.covariance.data[3] = sacc_var; // Vn
  fix.covariance.data[4] = sacc_var; // Ve
  fix.covariance.data[5] = sacc_var; // Vd
  fix.pdop = static_cast<float>(msg.pDOP) * 0.01F;
  fix.ecef_position_velocity.len = 0;

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

  if (self->lastNavSatITOW != msg.iTOW) {
    self->satsVisible = 0;
    self->lastNavSatITOW = msg.iTOW;
  }

  self->satsVisible += msg.numSvs;

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




namespace {
  constexpr bool isLeapYear(uint16_t year)
  {
    return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
  }

  // Convert UBX UTC date/time to Unix epoch usec. Return false if invalid.
  bool utcToUnixUsec(const UBX::NavPvt& msg, uint64_t& out)
  {
    if (!msg.valid.validDate || !msg.valid.validTime) {
      return false;
    }
    if (msg.year < 1970 || msg.month < 1 || msg.month > 12 || msg.day < 1 || msg.day > 31) {
      return false;
    }

    static constexpr uint8_t daysInMonth[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    const bool leap = isLeapYear(msg.year);
    const uint8_t dim = daysInMonth[msg.month - 1] + ((msg.month == 2 && leap) ? 1 : 0);
    if (msg.day > dim) {
      return false;
    }

    uint32_t days = 0;
    for (uint16_t y = 1970; y < msg.year; ++y) {
      days += isLeapYear(y) ? 366U : 365U;
    }
    for (uint8_t m = 1; m < msg.month; ++m) {
      days += daysInMonth[m - 1];
      if (m == 2 && leap) {
        ++days;
      }
    }
    days += static_cast<uint32_t>(msg.day - 1U);

    const uint64_t secInDay = static_cast<uint64_t>(msg.hour) * 3600ULL
      + static_cast<uint64_t>(msg.min) * 60ULL
      + static_cast<uint64_t>(msg.sec);
    const uint64_t baseUsec = (static_cast<uint64_t>(days) * 86400ULL + secInDay) * 1'000'000ULL;

    // nano can be negative in UBX to represent fractional seconds before the sec tick.
    const int64_t nano = msg.nano;
    const int64_t nano_to_usec = nano / 1000;
    const int64_t usecSigned = static_cast<int64_t>(baseUsec) + nano_to_usec;
    if (usecSigned < 0) {
      return false;
    }

    out = static_cast<uint64_t>(usecSigned);
    return true;
  }

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
// Dans le subscribe : rien à faire : on ne reçoit pas de messages UAVCan

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
  
  chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(1536), "gps", NORMALPRIO, 
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

IN_DMA_SECTION(uint8_t GpsUBX::frame[maxUbxFrameSize]);
