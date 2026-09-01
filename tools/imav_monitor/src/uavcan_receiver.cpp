#include "uavcan_receiver.hpp"

#include <uavcan.protocol.debug.KeyValue.h>

#include <chrono>
#include <utility>

namespace {

uint64_t monotonicMicros() {
  using namespace std::chrono;
  return static_cast<uint64_t>(duration_cast<microseconds>(
    steady_clock::now().time_since_epoch()).count());
}

} // namespace

bool UavcanReceiver::open(const std::string& interfaceName,
                          uint8_t sourceNodeId,
                          std::string& error) {
  close();
  if (sourceNodeId < 1U || sourceNodeId > 127U) {
    error = "source node id must be in [1..127]";
    lastError_ = error;
    return false;
  }
  if (!socket_.open(interfaceName, error)) {
    lastError_ = error;
    return false;
  }

  sourceNodeId_ = sourceNodeId;
  canardMemory_.assign(64U * 1024U, 0U);
  canardInit(&canard_, canardMemory_.data(), canardMemory_.size(),
             &UavcanReceiver::onTransferReceived,
             &UavcanReceiver::shouldAcceptTransfer, this);
  canFrameCount_ = 0;
  acceptedValueCount_ = 0;
  lastCleanupMicros_ = monotonicMicros();
  lastError_.clear();
  return true;
}

void UavcanReceiver::close() {
  socket_.close();
  canardMemory_.clear();
  sourceNodeId_ = 0;
  lastCleanupMicros_ = 0;
}

bool UavcanReceiver::isOpen() const { return socket_.isOpen(); }

bool UavcanReceiver::pump() {
  if (!socket_.isOpen()) {
    lastError_ = "CAN socket is not open";
    return false;
  }

  while (true) {
    CanardCANFrame frame{};
    const int result = socket_.receive(frame);
    if (result < 0) {
      lastError_ = socket_.lastError();
      return false;
    }
    if (result == 0) {
      break;
    }
    ++canFrameCount_;
    (void) canardHandleRxFrame(&canard_, &frame, monotonicMicros());
  }

  const uint64_t now = monotonicMicros();
  if (now - lastCleanupMicros_ >=
      CANARD_RECOMMENDED_STALE_TRANSFER_CLEANUP_INTERVAL_USEC) {
    canardCleanupStaleTransfers(&canard_, now);
    lastCleanupMicros_ = now;
  }
  return true;
}

void UavcanReceiver::setCallback(Callback callback) {
  callback_ = std::move(callback);
}

uint64_t UavcanReceiver::canFrameCount() const { return canFrameCount_; }

uint64_t UavcanReceiver::acceptedValueCount() const {
  return acceptedValueCount_;
}

const std::string& UavcanReceiver::lastError() const { return lastError_; }

void UavcanReceiver::onTransferReceived(CanardInstance* instance,
                                        CanardRxTransfer* transfer) {
  auto* self = static_cast<UavcanReceiver*>(instance->user_reference);
  self->handleTransfer(*transfer);
  canardReleaseRxTransferPayload(instance, transfer);
}

bool UavcanReceiver::shouldAcceptTransfer(const CanardInstance*,
                                          uint64_t* dataTypeSignature,
                                          uint16_t dataTypeId,
                                          CanardTransferType transferType,
                                          uint8_t) {
  if (transferType == CanardTransferTypeBroadcast &&
      dataTypeId == UAVCAN_PROTOCOL_DEBUG_KEYVALUE_ID) {
    *dataTypeSignature = UAVCAN_PROTOCOL_DEBUG_KEYVALUE_SIGNATURE;
    return true;
  }
  return false;
}

void UavcanReceiver::handleTransfer(const CanardRxTransfer& transfer) {
  if (transfer.source_node_id != sourceNodeId_ ||
      transfer.transfer_type != CanardTransferTypeBroadcast ||
      transfer.data_type_id != UAVCAN_PROTOCOL_DEBUG_KEYVALUE_ID) {
    return;
  }

  uavcan_protocol_debug_KeyValue message{};
  if (uavcan_protocol_debug_KeyValue_decode(&transfer, &message)) {
    return;
  }

  ++acceptedValueCount_;
  if (callback_) {
    callback_({transfer.source_node_id,
               std::string(reinterpret_cast<const char*>(message.key.data),
                           message.key.len),
               message.value});
  }
}
