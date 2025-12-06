#include "roleConf.h"

#if USE_GPS_UBX_ROLE

#include "gpsUbxDecoder.hpp"
#include "stdutil.h"
#include <cstring>

using namespace UBX;


void Decoder::reset()
{
  payload.clear();
  expectedLength = 0;
  cls = 0;
  id = 0;
  ckA = 0;
  ckB = 0;
  recvCkA = 0;
  recvCkB = 0;
  state = WAIT_FOR_SYNC1;
}

inline void Decoder::updateChecksum(uint8_t byte)
{
  ckA = ckA + byte;
  ckB = ckB + ckA;
}

bool Decoder::dispatch()
{
  if (cls != static_cast<uint8_t>(MessageClass::NAV)) {
    return false;
  }

  if (id == static_cast<uint8_t>(NavId::PVT) &&
      payload.size() == sizeof(NavPvt) &&
      cfg.navPvtCb) {
    const auto* msg = reinterpret_cast<const NavPvt*>(payload.data());
    return  cfg.navPvtCb(*msg);
  }
  
  if (id == static_cast<uint8_t>(NavId::DOP) &&
      payload.size() == sizeof(NavDop) &&
      cfg.navDopCb) {
    const auto* msg = reinterpret_cast<const NavDop*>(payload.data());
     return cfg.navDopCb(*msg);
  }
  
  if (id == static_cast<uint8_t>(NavId::SAT) &&
      payload.size() >= sizeof(NavSat) &&
      cfg.navSatCb) {
    const auto* sat = reinterpret_cast<const NavSat*>(payload.data());
    const std::size_t headerSize = sizeof(NavSat);
    if (payload.size() < headerSize) {
      return false;
    }
    const std::size_t svArea = payload.size() - headerSize;
    if (svArea % sizeof(NavSatSv) != 0) {
      return false;
    }
    const std::size_t svCount = svArea / sizeof(NavSatSv);
    if (sat->numSvs != svCount) {
      return false;
    }
    return cfg.navSatCb(*sat);
  }

  DebugTrace("Error : Don't knwow how to DISPATCH type %u with size %u",
	     id, payload.size());

  return false;
}

void UBX::Decoder::feed(etl::span<const uint8_t> data)
{
  std::size_t idx = 0;
  const std::size_t total = data.size();

  while (idx < total) {
    const uint8_t byte = data[idx];
    switch (state) {
    case WAIT_FOR_SYNC1:
      //      DebugTrace("UBX FSM: WAIT_FOR_SYNC1 byte=0x%02X", byte);
      if (byte == Sync1) {
        reset();
        state = WAIT_FOR_SYNC2;
      }
      ++idx;
      break;

    case WAIT_FOR_SYNC2:
      //      DebugTrace("UBX FSM: WAIT_FOR_SYNC2 byte=0x%02X", byte);
      if (byte == Sync2) {
        state = WAIT_FOR_CLASS;
      } else if (byte == Sync1) {
	// sliding window check :
        // stay in WAIT_FOR_SYNC2 to allow immediate resync
        state = WAIT_FOR_SYNC2;
      } else {
        state = WAIT_FOR_SYNC1;
      }
      ++idx;
      break;

    case WAIT_FOR_CLASS:
      //      DebugTrace("UBX FSM: WAIT_FOR_CLASS cls=0x%02X", byte);
      cls = byte;
      ckA = ckB = 0;
      updateChecksum(byte);
      state = WAIT_FOR_ID;
      ++idx;
      break;

    case WAIT_FOR_ID:
      // DebugTrace("UBX FSM: WAIT_FOR_ID id=0x%02X", byte);
      id = byte;
      updateChecksum(byte);
      state = WAIT_FOR_LEN1;
      ++idx;
      break;

    case WAIT_FOR_LEN1:
      // DebugTrace("UBX FSM: WAIT_FOR_LEN1 len_lsb=%u", static_cast<unsigned>(byte));
      expectedLength = byte;
      updateChecksum(byte);
      state = WAIT_FOR_LEN2;
      ++idx;
      break;

    case WAIT_FOR_LEN2:
      // DebugTrace("UBX FSM: WAIT_FOR_LEN2 len_msb=%u (len=%u)", static_cast<unsigned>(byte), static_cast<unsigned>(expectedLength | (static_cast<uint16_t>(byte) << 8)));
      expectedLength |= static_cast<uint16_t>(byte) << 8;
      updateChecksum(byte);
      payload.clear();
      if (expectedLength > payload.max_size()) {
        reset();
        if (byte == Sync1) {
          state = WAIT_FOR_SYNC2;
        }
      } else {
        state = (expectedLength == 0) ? WAIT_FOR_CHECKSUM_1 : PROCESSING_PAYLOAD;
      }
      ++idx;
      break;

    case PROCESSING_PAYLOAD:
    {
      const std::size_t already = payload.size();
      const std::size_t remaining = expectedLength - already;
      const std::size_t available = total - idx;
      const std::size_t chunk = (available < remaining) ? available : remaining;

      if (chunk > 0) {
        // DebugTrace("UBX FSM: PROCESS payload chunk=%u remaining_before=%u", static_cast<unsigned>(chunk), static_cast<unsigned>(remaining));
        const std::size_t oldSize = payload.size();
        payload.resize(oldSize + chunk);
        std::memcpy(payload.data() + oldSize, data.data() + idx, chunk);
        for (std::size_t i = 0; i < chunk; ++i) {
          updateChecksum(data[idx + i]);
        }
        idx += chunk;
      }

      if (payload.size() >= expectedLength) {
        state = WAIT_FOR_CHECKSUM_1;
      }
      break;
    }

    case WAIT_FOR_CHECKSUM_1:
      // DebugTrace("UBX FSM: WAIT_FOR_CHECKSUM_1 ckA=0x%02X", byte);
      recvCkA = byte;
      state = WAIT_FOR_CHECKSUM_2;
      ++idx;
      break;

    case WAIT_FOR_CHECKSUM_2:
      // DebugTrace("UBX FSM: WAIT_FOR_CHECKSUM_2 ckB=0x%02X (calc:0x%02X/0x%02X)", byte, ckA, ckB);
      recvCkB = byte;
      if (recvCkA == ckA && recvCkB == ckB) {
        if (not dispatch()) {
	  DebugTrace("ERROR : Decoder::dispatch failed");
	} else {
	  // DebugTrace("INFO : Decoder::dispatch succeed");
	}
      } else {
        DebugTrace("Checksum error");
      }
      reset();
      ++idx;
      break;
    }
  }
}

#endif // USE_GPS_UBX_ROLE