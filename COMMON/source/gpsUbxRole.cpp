/**
 * @file gpsUbxRole.cpp
 * @brief UBX-backed GPS role implementation.
 */
#include "roleConf.h"

#if USE_GPS_UBX_ROLE

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>

#include "gpsUbxRole.hpp"
#include "hardwareConf.hpp"
#include "resourceManager.hpp"
#include "stdutil++.hpp"
#include "etl/span.h"

#if PLATFORM_MICROCAN
#include "dynamicPinConfig.hpp"
#endif

namespace {
  /**
   * @brief Convert UBX UTC date/time to Unix epoch microseconds.
   * @return True when the date/time fields are valid.
   */
  bool utcToUnixUsec(const UBX::NavPvt& msg, uint64_t& out);

#if PLATFORM_MINICAN
  /**
   * @brief UBX framing and configuration constants (M10 compatible).
   *
   * @note These keys match u-blox VALSET identifiers used by AP_GPS_UBLOX.
   */
  constexpr uint8_t ubxSync1 = 0xB5;
  constexpr uint8_t ubxSync2 = 0x62;
  constexpr uint8_t ubxClassCfg = 0x06;
  constexpr uint8_t ubxIdCfgPrt = 0x00;
  constexpr uint8_t ubxIdCfgMsg = 0x01;
  constexpr uint8_t ubxIdCfgRate = 0x08;
  constexpr uint8_t ubxIdCfgValset = 0x8A;

  constexpr uint8_t ubxValsetLayerRam = 0x01;
  constexpr uint8_t ubxValsetLayerBbr = 0x02;

  constexpr uint32_t ubxCfgUart1Baudrate = 0x40520001;
  constexpr uint32_t ubxCfgUart1Enabled = 0x10520005;
  constexpr uint32_t ubxCfgUart1InProtUbx = 0x10730001;
  constexpr uint32_t ubxCfgUart1InProtNmea = 0x10730002;
  constexpr uint32_t ubxCfgUart1OutProtUbx = 0x10740001;
  constexpr uint32_t ubxCfgUart1OutProtNmea = 0x10740002;
  constexpr uint32_t ubxCfgRateMeas = 0x30210001;
  constexpr uint32_t ubxCfgNavspgDynModel = 0x20110021;

  /**
   * @brief Return the value size encoded in a VALSET key.
   */
  uint8_t ubxKeySize(uint32_t key)
  {
    const uint8_t keySize = (key >> 28) & 0x07;
    switch (keySize) {
    case 0x1:
    case 0x2:
      return 1;
    case 0x3:
      return 2;
    case 0x4:
      return 4;
    case 0x5:
      return 8;
    default:
      return 0;
    }
  }

  /**
   * @brief Update UBX checksum over a byte span.
   */
  void ubxUpdateChecksum(etl::span<const uint8_t> data, uint8_t &ckA, uint8_t &ckB)
  {
    for (const uint8_t byte : data) {
      ckA = static_cast<uint8_t>(ckA + byte);
      ckB = static_cast<uint8_t>(ckB + ckA);
    }
  }

  /**
   * @brief Send a raw UBX message (header + payload + checksum).
   */
  void ubxSend(UARTDriver &uart, uint8_t cls, uint8_t id, etl::span<const uint8_t> payload)
  {
    const size_t payloadLen = payload.size();
    uint8_t header[6] = {
      ubxSync1,
      ubxSync2,
      cls,
      id,
      static_cast<uint8_t>(payloadLen & 0xFF),
      static_cast<uint8_t>((payloadLen >> 8) & 0xFF),
    };

    uint8_t ckA = 0;
    uint8_t ckB = 0;
    ubxUpdateChecksum(etl::span<const uint8_t>(header + 2, 4), ckA, ckB);
    if (payloadLen != 0) {
      ubxUpdateChecksum(payload, ckA, ckB);
    }

    size_t toSend = sizeof(header);
    uartSendTimeout(&uart, &toSend, header, TIME_INFINITE);
    if (payloadLen != 0) {
      toSend = payloadLen;
      uartSendTimeout(&uart, &toSend, payload.data(), TIME_INFINITE);
    }
    uint8_t ck[2] = {ckA, ckB};
    toSend = sizeof(ck);
    uartSendTimeout(&uart, &toSend, ck, TIME_INFINITE);
  }

  /**
   * @brief Configure UART1 port settings (legacy CFG-PRT).
   *
   * @note Uses UBX-only protocol with 8N1 framing.
   */
  void ubxSendCfgPrt(UARTDriver &uart, uint32_t baudrate)
  {
    UBX::CfgPrt msg{};
    msg.portId = 1; // UART1
    msg.mode = 0x000008D0; // 8N1
    msg.baudRate = baudrate;
    msg.inProtoMask = 0x0001; // UBX only
    msg.outProtoMask = 0x0001; // UBX only
    ubxSend(uart, ubxClassCfg, ubxIdCfgPrt,
            etl::span<const uint8_t>(reinterpret_cast<const uint8_t *>(&msg), sizeof(msg)));
  }

  /**
   * @brief Configure measurement rate (CFG-RATE).
   */
  void ubxSendCfgRate(UARTDriver &uart, uint16_t measRateMs)
  {
    UBX::CfgRate msg{};
    msg.measRateMs = measRateMs;
    msg.navRate = 1;
    msg.timeRef = 1; // GPS time
    ubxSend(uart, ubxClassCfg, ubxIdCfgRate,
            etl::span<const uint8_t>(reinterpret_cast<const uint8_t *>(&msg), sizeof(msg)));
  }

  /**
   * @brief Configure output rate of a NAV message (legacy CFG-MSG).
   */
  void ubxSendCfgMsgRate(UARTDriver &uart, uint8_t msgClass, uint8_t msgId, uint8_t rate)
  {
    UBX::CfgMsgRate msg{};
    msg.msgClass = msgClass;
    msg.msgId = msgId;
    msg.rate = rate;
    ubxSend(uart, ubxClassCfg, ubxIdCfgMsg,
            etl::span<const uint8_t>(reinterpret_cast<const uint8_t *>(&msg), sizeof(msg)));
  }

  /**
   * @brief Send a single VALSET key/value pair.
   */
  void ubxSendValset(UARTDriver &uart, uint32_t key, const void *value, uint8_t valueLen)
  {
    if (valueLen == 0) {
      return;
    }
    UBX::CfgValset msg{};
    msg.version = 1;
    msg.layers = ubxValsetLayerRam | ubxValsetLayerBbr;
    msg.key = key;

    uint8_t buffer[sizeof(msg) + 8] = {};
    std::memcpy(buffer, &msg, sizeof(msg));
    std::memcpy(buffer + sizeof(msg), value, valueLen);
    ubxSend(uart, ubxClassCfg, ubxIdCfgValset,
            etl::span<const uint8_t>(buffer, sizeof(msg) + valueLen));
  }

  /**
   * @brief Minimal legacy configuration (CFG-RATE/CFG-MSG/CFG-PRT).
   *
   * @note This is the fallback when VALSET is not supported.
   */
  void ubxSendConfigLegacy(UARTDriver &uart, uint32_t baudrate, uint16_t measRateMs)
  {
    ubxSendCfgRate(uart, measRateMs);
    ubxSendCfgMsgRate(uart, static_cast<uint8_t>(UBX::MessageClass::NAV),
                      static_cast<uint8_t>(UBX::NavId::PVT), 1);
    ubxSendCfgMsgRate(uart, static_cast<uint8_t>(UBX::MessageClass::NAV),
                      static_cast<uint8_t>(UBX::NavId::DOP), 1);
    ubxSendCfgMsgRate(uart, static_cast<uint8_t>(UBX::MessageClass::NAV),
                      static_cast<uint8_t>(UBX::NavId::SAT), 1);

    ubxSendCfgPrt(uart, baudrate);
  }

  /**
   * @brief M10-compatible configuration using VALSET keys.
   *
   * @note Forces UBX protocol, disables NMEA, and selects a 10 Hz rate.
   */
  void ubxSendConfigValset(UARTDriver &uart, uint32_t baudrate, uint16_t measRateMs)
  {
    ubxSendCfgRate(uart, measRateMs);
    ubxSendCfgMsgRate(uart, static_cast<uint8_t>(UBX::MessageClass::NAV),
                      static_cast<uint8_t>(UBX::NavId::PVT), 1);
    ubxSendCfgMsgRate(uart, static_cast<uint8_t>(UBX::MessageClass::NAV),
                      static_cast<uint8_t>(UBX::NavId::DOP), 1);
    ubxSendCfgMsgRate(uart, static_cast<uint8_t>(UBX::MessageClass::NAV),
                      static_cast<uint8_t>(UBX::NavId::SAT), 1);

    const uint8_t ubxOn = 1;
    const uint8_t nmeaOff = 0;
    const uint8_t uartEnabled = 1;
    const uint8_t dynModelAirborne = 8; // air < 4g
    const uint8_t rateSize = ubxKeySize(ubxCfgRateMeas);
    const uint8_t baudSize = ubxKeySize(ubxCfgUart1Baudrate);
    if (baudSize == sizeof(uint32_t)) {
      ubxSendValset(uart, ubxCfgUart1Baudrate, &baudrate, baudSize);
    }
    ubxSendValset(uart, ubxCfgUart1Enabled, &uartEnabled, ubxKeySize(ubxCfgUart1Enabled));
    ubxSendValset(uart, ubxCfgUart1InProtUbx, &ubxOn, ubxKeySize(ubxCfgUart1InProtUbx));
    ubxSendValset(uart, ubxCfgUart1InProtNmea, &nmeaOff, ubxKeySize(ubxCfgUart1InProtNmea));
    ubxSendValset(uart, ubxCfgUart1OutProtUbx, &ubxOn, ubxKeySize(ubxCfgUart1OutProtUbx));
    ubxSendValset(uart, ubxCfgUart1OutProtNmea, &nmeaOff, ubxKeySize(ubxCfgUart1OutProtNmea));
    if (rateSize == sizeof(uint16_t)) {
      ubxSendValset(uart, ubxCfgRateMeas, &measRateMs, rateSize);
    }
    ubxSendValset(uart, ubxCfgNavspgDynModel, &dynModelAirborne,
                  ubxKeySize(ubxCfgNavspgDynModel));
  }

  /**
   * @brief Scan for UBX sync bytes to confirm the link is alive.
   */
  bool waitForUbxSync(UARTDriver &uart, systime_t timeout)
  {
    systime_t start = chVTGetSystemTimeX();
    uint8_t prev = 0;
    uint8_t buffer[64];

    while (chTimeDiffX(start, chVTGetSystemTimeX()) < timeout) {
      size_t size = sizeof(buffer);
      uartReceiveTimeout(&uart, &size, buffer, TIME_MS2I(50));
      for (size_t i = 0; i < size; ++i) {
        const uint8_t byte = buffer[i];
        if (prev == ubxSync1 && byte == ubxSync2) {
          return true;
        }
        prev = byte;
      }
    }
    return false;
  }

  /**
   * @brief Configure the GPS over UART and switch to the target baudrate.
   *
   * @details Tries the target baud first, then common fallbacks. Attempts
   *          VALSET configuration first (M10), then legacy CFG-*.
   */
  void configureGpsUbx(UARTDriver &uart, UARTConfig &cfg, uint32_t targetBaud)
  {
    static constexpr uint16_t measRateMs = 100;
    static constexpr std::array<uint32_t, 6> baudList = {
      9600, 38400, 57600, 115200, 230400, 460800
    };

    auto tryConfig = [&](uint32_t baud) -> bool {
      cfg.speed = baud;
      uartStop(&uart);
      uartStart(&uart, &cfg);
      ubxSendConfigValset(uart, targetBaud, measRateMs);
      if (baud != targetBaud) {
        cfg.speed = targetBaud;
        uartStop(&uart);
        uartStart(&uart, &cfg);
        chThdSleepMilliseconds(50);
      }
      if (waitForUbxSync(uart, TIME_MS2I(300))) {
        return true;
      }

      cfg.speed = baud;
      uartStop(&uart);
      uartStart(&uart, &cfg);
      ubxSendConfigLegacy(uart, targetBaud, measRateMs);
      if (baud != targetBaud) {
        cfg.speed = targetBaud;
        uartStop(&uart);
        uartStart(&uart, &cfg);
        chThdSleepMilliseconds(50);
      }
      return waitForUbxSync(uart, TIME_MS2I(300));
    };

    if (tryConfig(targetBaud)) {
      return;
    }
    for (const auto baud : baudList) {
      if (baud == targetBaud) {
        continue;
      }
      if (tryConfig(baud)) {
        return;
      }
    }
  }
#endif // PLATFORM_MINICAN
}

/**
 * @brief Handle UBX NAV-PVT and publish Fix2 + ardupilot.gnss.Status.
 */
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

  const bool fixSent = self->m_node->sendBroadcast(fix) == UAVCAN::Node::CAN_OK;

  const bool fixOk = msg.flags.gnssFixOK != 0;
  const bool healthy = fixOk && (fix.status == UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX);
  const float hAcc_m = static_cast<float>(msg.hAcc) * 0.001F;
  const float vAcc_m = static_cast<float>(msg.vAcc) * 0.001F;
  const float sAcc_mps = static_cast<float>(msg.sAcc) * 0.001F;
  const bool dopOk = !self->dopCache.valid || (self->dopCache.hdop <= 2.5F);
  const bool accOk = (hAcc_m <= 5.0F) && (vAcc_m <= 7.5F) && (sAcc_mps <= 1.0F);
  const bool armable = healthy && dopOk && accOk;

  ardupilot_gnss_Status status{};
  status.healthy = healthy;
  if (armable) {
    status.status |= ARDUPILOT_GNSS_STATUS_STATUS_ARMABLE;
  }
  self->m_node->sendBroadcast(status);

  return fixSent;
}

/**
 * @brief Handle UBX NAV-DOP and publish Auxiliary.
 */
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

/**
 * @brief Handle UBX NAV-SAT and publish Auxiliary with satellite counts.
 */
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

/**
 * @brief Construct the UBX role and bind decoder callbacks.
 */
GpsUBX::GpsUBX()
  : decoder(config)
{
}




namespace {
  /**
   * @brief Check if a year is a leap year.
   */
  constexpr bool isLeapYear(uint16_t year)
  {
    return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
  }

  /**
   * @brief Convert UBX UTC date/time to Unix epoch microseconds.
   * @return False if the date/time is invalid.
   */
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

/**
 * @brief GPS role requires no subscriptions; return role status.
 */
DeviceStatus GpsUBX::subscribe(UAVCAN::Node&)
{
  return DeviceStatus(DeviceStatus::GPS_ROLE);
}


/**
 * @brief Start UART reception and the UBX decoder thread.
 */
DeviceStatus GpsUBX::start(UAVCAN::Node& node)
{
  using HR = HWResource;
  DeviceStatus status(DeviceStatus::GPS_ROLE);
  m_node = &node;
  
#if PLATFORM_MINICAN
  if (not boardResource.tryAcquire(HR::USART_2, HR::PB03, HR::PB04)) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT,
			std::to_underlying(HR::USART_2));
  }
#endif
  
#if PLATFORM_MICROCAN
  if (not boardResource.tryAcquire(HR::USART_1, HR::PA10, HR::F3)) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT,
			std::to_underlying(HR::USART_1));
  }
  //  MSB F4 F3 F2a F0b F0a LSB
  DynPin::setScenario(DynPin::Scenario::UART, 0b01000);
#endif

  frame = (uint8_t *) malloc_dma(maxUbxFrameSize);
  if (!frame) {
    status = DeviceStatus(DeviceStatus::MEMORY, DeviceStatus::DMA_HEAP_FULL);
    return status;
  }

  gpscfg.speed =  param_cget<"bus.serial.baudrate">();
#if PLATFORM_MINICAN
  // minican is full duplex for the GPS and can configure it
  // using ublox protocol
  configureGpsUbx(ExternalUARTD, gpscfg, gpscfg.speed);
#else
  // microcan is RX only due to smaller connector and expect an already
  // configured gps via u‑center (newer version: u‑center2 for M10+)
  uartStart(&ExternalUARTD, &gpscfg);
#endif

  chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(1536), "gps", NORMALPRIO, 
		      &Trampoline<&GpsUBX::periodic>::fn, this);
  
  return status;
}

/**
 * @brief Thread loop that reads UART frames and feeds the decoder.
 */
void GpsUBX::periodic(void *)
{
  size_t size;

  while (true) {
    size = maxUbxFrameSize;
    uartReceiveTimeout(&ExternalUARTD, &size, frame, TIME_INFINITE);
    if (size) {
      decoder.feed({frame, size});
    }
  }
}


/*
  en statique :
  p &frame
  $2 = (uint8_t (*)[1024]) 0x20008c58 <GpsUBX::frame>
  en dynamique : 
  p frame
  $2 = (uint8_t *) 0x200004a0 'U' <repeats 200 times>...
*/
//IN_DMA_SECTION(uint8_t GpsUBX::frame[maxUbxFrameSize]);

#endif // USE_GPS_UBX_ROLE
