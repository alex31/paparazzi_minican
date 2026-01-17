/**
 * @file gpsUbxMessageDef.hpp
 * @brief UBX message identifiers and payload layouts.
 */
#pragma once

#include <cstdint>

namespace UBX {

  /** @brief UBX message classes (u-blox M10 reference). */
  enum class MessageClass : std::uint8_t {
    NAV = 0x01, RXM = 0x02, INF = 0x04, ACK = 0x05,
    CFG = 0x06, UPD = 0x09, MON = 0x0A, TIM = 0x0D,
    ESF = 0x10, MGA = 0x13, LOG = 0x21, SEC = 0x27,
    HNR = 0x28,
  };

  /** @brief NAV message IDs (M10 output). */
  enum class NavId : std::uint8_t {
    POSECEF = 0x01, POSLLH = 0x02, STATUS = 0x03, DOP = 0x04,
    ATT = 0x05, SOL = 0x06, PVT = 0x07, ODO = 0x09,
    VELECEF = 0x11, VELNED = 0x12, HPPOSECEF = 0x13, HPPOSLLH = 0x14,
    TIMEGPS = 0x20, TIMEUTC = 0x21, CLOCK = 0x22, TIMEGLO = 0x23,
    TIMEBDS = 0x24, TIMEGAL = 0x25, TIMELS = 0x26,
    ORB = 0x34, SAT = 0x35, COV = 0x36, RELPOSNED = 0x3C,
    SIG = 0x43, EOE = 0x61,
  };

  /** @brief RXM message IDs. */
  enum class RxmId : std::uint8_t {
    SFRBX = 0x13, MEASX = 0x14, RAWX = 0x15,
    RLM = 0x59, PMREQ = 0x41,
  };

  /** @brief MON message IDs. */
  enum class MonId : std::uint8_t {
    IO = 0x02, VER = 0x04, MSGPP = 0x06, RXBUF = 0x07,
    TXBUF = 0x08, HW = 0x09, HW2 = 0x0B, PATCH = 0x27,
    GNSS = 0x28, RF = 0x38,
  };

  /** @brief TIM message IDs. */
  enum class TimId : std::uint8_t {
    TP = 0x01, TM2 = 0x03, SVIN = 0x04,
    VCOCAL = 0x0A, FCHG = 0x10, HOC = 0x17,
  };

  /** @brief ESF (external sensor fusion) message IDs. */
  enum class EsfId : std::uint8_t {
    MEAS = 0x02, RAW = 0x03, STATUS = 0x10, INS = 0x15,
  };

  /** @brief INF (text) message IDs. */
  enum class InfId : std::uint8_t {
    ERROR = 0x00, WARNING = 0x01, NOTICE = 0x02,
    TEST = 0x03, DEBUG = 0x04, USER = 0x08,
  };

  /** @brief ACK message IDs. */
  enum class AckId : std::uint8_t {
    NAK = 0x00, ACK = 0x01,
  };

  /** @brief MGA (GNSS assistance) message IDs. */
  enum class MgaId : std::uint8_t {
    ANO = 0x20, BCH = 0x28, BDS = 0x03, GAL = 0x02,
    GLO = 0x06, GPS = 0x00, QZSS = 0x05, TIM = 0x10,
    MFLASH = 0x21,
  };

  /** @brief LOG message IDs. */
  enum class LogId : std::uint8_t {
    ERASELOG = 0x00, STRING = 0x04, INFO = 0x08, CREATE = 0x07,
    RETRIEVEPOS = 0x0B, RETRIEVESTRING = 0x0D, FINDTIME = 0x0E,
    RETRIEVEBATCH = 0x10,
  };

  /** @brief SEC message IDs. */
  enum class SecId : std::uint8_t {
    SIGN = 0x01, UNIQID = 0x03, VER = 0x09,
  };

  /** @brief HNR (high navigation rate) message IDs. */
  enum class HnrId : std::uint8_t {
    PVT = 0x00,
  };

  /** @brief NAV-PVT fix type (UBX-13003221). */
  enum class NavFixType : std::uint8_t {
    NO_FIX = 0, DEAD_RECKONING_ONLY = 1, TWO_D = 2,
    THREE_D = 3, GNSS_AND_DR = 4, TIME_ONLY = 5
  };

  /** @brief UBX-CFG-PRT payload for UART port configuration. */
  struct CfgPrt {
    std::uint8_t portId;
    std::uint8_t reserved0;
    std::uint16_t txReady;
    std::uint32_t mode;
    std::uint32_t baudRate;
    std::uint16_t inProtoMask;
    std::uint16_t outProtoMask;
    std::uint16_t flags;
    std::uint16_t reserved5;
  } __attribute__((packed));

  /** @brief UBX-CFG-RATE payload for navigation rate. */
  struct CfgRate {
    std::uint16_t measRateMs;
    std::uint16_t navRate;
    std::uint16_t timeRef;
  } __attribute__((packed));

  /** @brief UBX-CFG-MSG payload (legacy per-message rate). */
  struct CfgMsgRate {
    std::uint8_t msgClass;
    std::uint8_t msgId;
    std::uint8_t rate;
  } __attribute__((packed));

  /** @brief UBX-CFG-VALSET header, followed by key/value pairs. */
  struct CfgValset {
    std::uint8_t version;
    std::uint8_t layers;
    std::uint8_t transaction;
    std::uint8_t reserved;
    std::uint32_t key;
  } __attribute__((packed));

  /** @brief NAV-PVT valid flags (byte 11). */
  struct NavPvtValid {
    std::uint8_t validDate     : 1; // bit0 : 1 = date UTC valide
    std::uint8_t validTime     : 1; // bit1 : 1 = heure UTC valide
    std::uint8_t fullyResolved : 1; // bit2 : 1 = pas d'incertitude sur les secondes
    std::uint8_t validMag      : 1; // bit3 : 1 = déclinaison magnétique valide
    std::uint8_t reserved      : 4;
  };
  static_assert(sizeof(NavPvtValid) == 1);
  /** @brief NAV-PVT flags (byte 21). */
  struct NavPvtFlags {
    std::uint8_t gnssFixOK   : 1; // bit0 : fix valide (dans les masques DOP/ACC)
    std::uint8_t diffSoln    : 1; // bit1 : corrections différentielles appliquées
    std::uint8_t psmState    : 3; // bits2..4 : état power-save mode
                                  //   0=ACQ/PSM off, 1=TRACK, 2=POT, 3=INACTIVE, etc.
    std::uint8_t headVehValid: 1; // bit5 : heading véhicule valide
    std::uint8_t carrSoln    : 2; // bits6..7 : solution phase porteuse
                                  //   0=pas de solution, 1=FLT, 2=FIX
  };
  static_assert(sizeof(NavPvtFlags) == 1);
  /** @brief NAV-PVT flags2 (byte 22). */
  struct NavPvtFlags2 {
    std::uint8_t reserved0      : 5;
    std::uint8_t confirmedAvai  : 1; // info de confirmation disponible
    std::uint8_t confirmedDate  : 1; // date UTC confirmée
    std::uint8_t confirmedTime  : 1; // heure UTC confirmée
  };
  static_assert(sizeof(NavPvtFlags2) == 1);
  /** @brief NAV-PVT flags3 (byte 78). */
  struct NavPvtFlags3 {
    std::uint8_t invalidLlh : 1; // 1 = lon/lat/height/hMSL invalides
    std::uint8_t reserved   : 7;
  };

  /** @brief UBX-NAV-PVT payload (0x01 0x07), length 92 bytes. */
  struct NavPvt {
    std::uint32_t iTOW;     // 0  U4   ms
    std::uint16_t year;     // 4  U2
    std::uint8_t  month;    // 6  U1
    std::uint8_t  day;      // 7  U1
    std::uint8_t  hour;     // 8  U1
    std::uint8_t  min;      // 9  U1
    std::uint8_t  sec;      //10  U1
    NavPvtValid   valid;    //11  X1
    std::uint32_t tAcc;     //12  U4   ns
    std::int32_t  nano;     //16  I4   ns
    std::uint8_t  fixType;  //20  U1
    NavPvtFlags   flags;    //21  X1
    NavPvtFlags2  flags2;   //22  X1
    std::uint8_t  numSV;    //23  U1

    std::int32_t  lon;      //24  I4   1e-7 deg
    std::int32_t  lat;      //28  I4   1e-7 deg
    std::int32_t  height;   //32  I4   mm (ellipsoïde)
    std::int32_t  hMSL;     //36  I4   mm (MSL)

    std::uint32_t hAcc;     //40  U4   mm
    std::uint32_t vAcc;     //44  U4   mm

    std::int32_t  velN;     //48  I4   mm/s
    std::int32_t  velE;     //52  I4   mm/s
    std::int32_t  velD;     //56  I4   mm/s
    std::int32_t  gSpeed;   //60  I4   mm/s
    std::int32_t  headMot;  //64  I4   1e-5 deg

    std::uint32_t sAcc;     //68  U4   mm/s
    std::uint32_t headAcc;  //72  U4   1e-5 deg

    std::uint16_t pDOP;     //76  U2   0.01
    NavPvtFlags3  flags3;   //78  X1
    std::uint8_t  reserved1[5]; //79 U1[5]

    std::int32_t  headVeh;  //84  I4   1e-5 deg
    std::int16_t  magDec;   //88  I2   1e-2 deg
    std::uint16_t magAcc;   //90  U2   1e-2 deg
  } __attribute__((packed));

  static_assert(sizeof(NavPvt) == 92, "UBX NAV-PVT size mismatch");

  /** @brief UBX-NAV-DOP payload (0x01 0x04), length 18 bytes. */
  struct NavDop {
    std::uint32_t iTOW;  // 0  U4   ms
    std::uint16_t gDOP;  // 4  U2   0.01
    std::uint16_t pDOP;  // 6  U2   0.01
    std::uint16_t tDOP;  // 8  U2   0.01
    std::uint16_t vDOP;  //10  U2   0.01
    std::uint16_t hDOP;  //12  U2   0.01
    std::uint16_t nDOP;  //14  U2   0.01
    std::uint16_t eDOP;  //16  U2   0.01
  } __attribute__((packed));

  static_assert(sizeof(NavDop) == 18, "UBX NAV-DOP size mismatch");

  /** @brief NAV-SAT satellite block flags. */
  struct NavSatFlags {
    std::uint32_t qualityInd : 3;  // bits0..2 : 0=no signal, 1=searching, 2=acquired, 3=unusable, 4=code lock, 5=code+carrier
    std::uint32_t svUsed     : 1;  // bit3      : satellite utilisé dans la solution nav
    std::uint32_t health     : 2;  // bit4..5   : O unknown, 1 = healthy, 2 unhealthy
    std::uint32_t diffCorr   : 1;  // bit6      : corrections différentielles appliquées
    std::uint32_t smoothed   : 1;  // bit7      : pseudo-distance lissée
    std::uint32_t orbitSource: 3;  // bits8..10 : 0=unknown, 1=eph, 2=alm, 3=assist, 4=aop, 5=decoded
    std::uint32_t ephAvail   : 1;  // bit11
    std::uint32_t almAvail   : 1;  // bit12
    std::uint32_t anoAvail   : 1;  // bit13
    std::uint32_t aopAvail   : 1;  // bit14
    std::uint32_t reserved1  : 1;  // bit15
    std::uint32_t sbasCorrUsed : 1; // bit16
    std::uint32_t rtcmCorrUsed : 1; // bit17
    std::uint32_t slasCorrUsed : 1; // bit18
    std::uint32_t spartnCorrUsed : 1; // bit19
    std::uint32_t prCorrUsed   : 1; // bit20
    std::uint32_t crCorrUsed   : 1; // bit21
    std::uint32_t doCorrUsed   : 1; // bit22
    std::uint32_t reserved3    : 9; // bits23..31
  };

  static_assert(sizeof(NavSatFlags) == 4);

  /** @brief NAV-SAT satellite block (12 bytes). */
  struct NavSatSv {
    std::uint8_t  gnssId; // 0  U1
    std::uint8_t  svId;   // 1  U1
    std::uint8_t  cno;    // 2  U1   dBHz
    std::int8_t   elev;   // 3  I1   deg
    std::int16_t  azim;   // 4  I2   deg
    std::int16_t  prRes;  // 6  I2   0.1 m
    NavSatFlags   flags;  // 8  X4
  } __attribute__((packed));

  static_assert(sizeof(NavSatSv) == 12, "UBX NAV-SAT SV size mismatch");

  /** @brief NAV-SAT header (0x01 0x35). */
  struct NavSat {
    std::uint32_t iTOW;      // 0 U4 ms
    std::uint8_t  version;   // 4 U1
    std::uint8_t  numSvs;    // 5 U1
    std::uint8_t  reserved0[2]; // 6 U1[2]
    // Extension GCC : tableau flexible, taille = numSvs (compter manuellement en C++)
    NavSatSv      svs[];     // suit numSvs blocs NavSatSv
  } __attribute__((packed));

  static_assert(sizeof(NavSat) == 8, "UBX NAV-SAT header size mismatch");



} // namespace UBX
