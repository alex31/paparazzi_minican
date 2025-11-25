#include "gpsUbxDecoder.hpp"
#include "stdutil.h"

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

void Decoder::dispatch()
{
  if (cls != static_cast<uint8_t>(MessageClass::NAV)) {
    return;
  }

  if (id == static_cast<uint8_t>(NavId::PVT) &&
      payload.size() == sizeof(NavPvt) &&
      cfg.navPvtCb) {
    const auto* msg = reinterpret_cast<const NavPvt*>(payload.data());
    cfg.navPvtCb(*msg);
    return;
  }

  if (id == static_cast<uint8_t>(NavId::DOP) &&
      payload.size() == sizeof(NavDop) &&
      cfg.navDopCb) {
    const auto* msg = reinterpret_cast<const NavDop*>(payload.data());
    cfg.navDopCb(*msg);
    return;
  }

  if (id == static_cast<uint8_t>(NavId::SAT) &&
      payload.size() >= sizeof(NavSat) &&
      cfg.navSatCb) {
    const auto* sat = reinterpret_cast<const NavSat*>(payload.data());
    const std::size_t headerSize = sizeof(NavSat);
    if (payload.size() < headerSize) {
      return;
    }
    const std::size_t svArea = payload.size() - headerSize;
    if (svArea % sizeof(NavSatSv) != 0) {
      return;
    }
    const std::size_t svCount = svArea / sizeof(NavSatSv);
    if (sat->numSvs != svCount) {
      return;
    }
    cfg.navSatCb(*sat, svCount);
    return;
  }

  DebugTrace("Error : Don't knwow how to DISPATCH type %u with size %u",
	     id, payload.size());
}

void UBX::Decoder::feed(etl::span<const uint8_t> data)
{
  for (uint8_t byte : data) {
    switch (state) {
    case WAIT_FOR_SYNC1:
      if (byte == Sync1) {
        reset();
        state = WAIT_FOR_SYNC2;
      }
      break;

    case WAIT_FOR_SYNC2:
      if (byte == Sync2) {
        state = WAIT_FOR_CLASS;
      } else if (byte == Sync1) {
        // stay in WAIT_FOR_SYNC2 to allow immediate resync
        state = WAIT_FOR_SYNC2;
      } else {
        state = WAIT_FOR_SYNC1;
      }
      break;

    case WAIT_FOR_CLASS:
      cls = byte;
      ckA = ckB = 0;
      updateChecksum(byte);
      state = WAIT_FOR_ID;
      break;

    case WAIT_FOR_ID:
      id = byte;
      updateChecksum(byte);
      state = WAIT_FOR_LEN1;
      break;

    case WAIT_FOR_LEN1:
      expectedLength = byte;
      updateChecksum(byte);
      state = WAIT_FOR_LEN2;
      break;

    case WAIT_FOR_LEN2:
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
      break;

    case PROCESSING_PAYLOAD:
      payload.push_back(byte);
      updateChecksum(byte);
      if (payload.size() >= expectedLength) {
        state = WAIT_FOR_CHECKSUM_1;
      }
      break;

    case WAIT_FOR_CHECKSUM_1:
      recvCkA = byte;
      state = WAIT_FOR_CHECKSUM_2;
      break;

    case WAIT_FOR_CHECKSUM_2:
      recvCkB = byte;
      if (recvCkA == ckA && recvCkB == ckB) {
        dispatch();
      } else {
	DebugTrace("Checksum error");
      }
      reset();
      break;
    }
  }
}
