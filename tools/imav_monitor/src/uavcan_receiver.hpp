#pragma once

#include "socket_can.hpp"

#include <canard.h>

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

struct ImavKeyValue {
  uint8_t sourceNodeId = 0;
  std::string key;
  float value = 0.0F;
};

/** Passive DroneCAN v0 receiver for uavcan.protocol.debug.KeyValue. */
class UavcanReceiver {
public:
  using Callback = std::function<void(const ImavKeyValue&)>;

  bool open(const std::string& interfaceName,
            uint8_t sourceNodeId,
            std::string& error);
  void close();
  bool isOpen() const;

  /** Drain all currently available frames. False means a SocketCAN error. */
  bool pump();
  void setCallback(Callback callback);

  uint64_t canFrameCount() const;
  uint64_t acceptedValueCount() const;
  const std::string& lastError() const;

private:
  static void onTransferReceived(CanardInstance* instance,
                                 CanardRxTransfer* transfer);
  static bool shouldAcceptTransfer(const CanardInstance* instance,
                                   uint64_t* dataTypeSignature,
                                   uint16_t dataTypeId,
                                   CanardTransferType transferType,
                                   uint8_t sourceNodeId);
  void handleTransfer(const CanardRxTransfer& transfer);

  SocketCan socket_;
  uint8_t sourceNodeId_ = 0;
  std::vector<uint8_t> canardMemory_;
  CanardInstance canard_{};
  Callback callback_;
  uint64_t canFrameCount_ = 0;
  uint64_t acceptedValueCount_ = 0;
  uint64_t lastCleanupMicros_ = 0;
  std::string lastError_;
};
