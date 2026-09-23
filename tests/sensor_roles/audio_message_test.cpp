#include "spectrumEncoding.hpp"
#include "spectrumMessage.hpp"
#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>

static void checkLevels()
{
  using namespace SpectrumEncoding;
  assert(std::abs(rangeDb(13U) - 72.2472f) < 0.0001f);
  assert(encodeLevel(-100.0f, 13U) == 0U);
  assert(encodeLevel(-rangeDb(13U), 13U) == 0U);
  assert(encodeLevel(0.0f, 13U) == 65535U);
  assert(encodeLevel(6.0f, 13U) == 65535U);
  assert(std::abs(int(encodeLevel(-dbPerBit, 13U)) - 60074) <= 1);
  assert(encodeLevel(std::numeric_limits<float>::quiet_NaN(), 13U) == 0U);
  assert(encodeLevel(0.0f, 1U) == 0U);
  for (uint8_t bits : {8U, 12U, 13U, 16U, 24U, 32U}) {
    uint16_t last = 0U;
    for (size_t i = 0U; i <= 8192U; ++i) {
      const float dbfs = rangeDb(bits) * (static_cast<float>(i) / 8192.0f - 1.0f);
      const uint16_t code = encodeLevel(dbfs, bits);
      assert(code >= last);
      assert(std::abs(decodeLevel(code, bits) - dbfs) <= rangeDb(bits) / 65535.0f);
      last = code;
    }
  }
  assert(encodeFrequency(984.28f) == 984U);
  assert(encodeFrequency(2999.72f) == 3000U);
  assert(encodeFrequency(70000.0f) == 65535U);
  assert(encodeFrequency(-1.0f) == 0U);
}

template<size_t Capacity>
static void checkCodec()
{
  SpectrumMessage<Capacity> compact;
  microcan_audio_Spectrum canonical = {};
  compact.adc_bits = canonical.adc_bits = 13U;
  compact.status = canonical.status = 7U;
  for (size_t i = 0U; i < Capacity; ++i) {
    compact.bins[i] = canonical.bins.data[i] = {
      static_cast<uint16_t>(65535U - 137U * i),
      static_cast<uint16_t>(65535U - 251U * i)};
  }
  std::array<uint8_t, MICROCAN_AUDIO_SPECTRUM_MAX_SIZE> full;
  // Canaries check that the small encoder never clears the protocol maximum.
  std::array<uint8_t, SpectrumMessage<Capacity>::cxx_iface::MAX_SIZE + 2U> bounded;
  for (size_t count = 0U; count <= Capacity; ++count) {
    compact.count = canonical.bins.len = static_cast<uint8_t>(count);
    for (bool tao : {true, false}) {
      bounded.fill(0xA5U);
      const auto size = microcan_audio_Spectrum_encode(&canonical, full.data(), tao);
      const auto compactSize = SpectrumMessage<Capacity>::cxx_iface::encode(
        &compact, bounded.data() + 1U, tao);
      assert(size == 2U + (tao ? 0U : 1U) + 4U * count);
      assert(compactSize == size && size <= CANARD_MAX_TRANSFER_PAYLOAD_LEN);
      assert(std::memcmp(full.data(), bounded.data() + 1U, size) == 0);
      assert(bounded.front() == 0xA5U && bounded.back() == 0xA5U);
      CanardRxTransfer transfer = {};
      transfer.payload_head = bounded.data() + 1U;
      transfer.payload_len = size;
      transfer.tao = tao;
      microcan_audio_Spectrum decoded = {};
      assert(not microcan_audio_Spectrum_decode(&transfer, &decoded));
      assert(decoded.adc_bits == 13U && decoded.status == 7U && decoded.bins.len == count);
      for (size_t i = 0U; i < count; ++i) {
        assert(decoded.bins.data[i].frequency_hz == compact.bins[i].frequency_hz);
        assert(decoded.bins.data[i].level == compact.bins[i].level);
      }
      --transfer.payload_len;
      assert(microcan_audio_Spectrum_decode(&transfer, &decoded));
    }
  }
}

static unsigned received;
static uint8_t expectedCount;
static void onReceive(CanardInstance*, CanardRxTransfer *transfer)
{
  microcan_audio_Spectrum decoded = {};
  assert(not microcan_audio_Spectrum_decode(transfer, &decoded));
  assert(decoded.adc_bits == 13U && decoded.status == 1U);
  assert(decoded.bins.len == expectedCount);
  for (size_t i = 0U; i < expectedCount; ++i) {
    assert(decoded.bins.data[i].frequency_hz == i * 251U);
    assert(decoded.bins.data[i].level == 65535U - i * 239U);
  }
  ++received;
}
static bool shouldAccept(const CanardInstance*, uint64_t *signature, uint16_t id,
                         CanardTransferType type, uint8_t)
{
  if (id != MICROCAN_AUDIO_SPECTRUM_ID || type != CanardTransferTypeBroadcast) {
    return false;
  }
  *signature = MICROCAN_AUDIO_SPECTRUM_SIGNATURE;
  return true;
}

// Exercise libcanard's actual fragmentation/reassembly, including CAN FD
// padding of a maximum-size transfer; application code does no fragmentation.
static void checkTransport(bool fd, uint8_t count)
{
  struct Pool { alignas(16) uint8_t data[65536]; };
  auto txPool = std::make_unique<Pool>();
  auto rxPool = std::make_unique<Pool>();
  CanardInstance tx, rx;
  canardInit(&tx, txPool->data, sizeof(txPool->data), onReceive, shouldAccept, nullptr);
  canardInit(&rx, rxPool->data, sizeof(rxPool->data), onReceive, shouldAccept, nullptr);
  canardSetLocalNodeID(&tx, 42U);
  canardSetLocalNodeID(&rx, 43U);
  microcan_audio_Spectrum message = {};
  message.adc_bits = 13U;
  message.status = 1U;
  message.bins.len = count;
  for (size_t i = 0U; i < count; ++i) {
    message.bins.data[i] = {static_cast<uint16_t>(i * 251U),
                            static_cast<uint16_t>(65535U - i * 239U)};
  }
  std::array<uint8_t, MICROCAN_AUDIO_SPECTRUM_MAX_SIZE> buffer;
  const uint16_t size = microcan_audio_Spectrum_encode(&message, buffer.data(), not fd);
  uint8_t transferId = 0U;
  CanardTxTransfer transfer = {};
  transfer.transfer_type = CanardTransferTypeBroadcast;
  transfer.data_type_signature = MICROCAN_AUDIO_SPECTRUM_SIGNATURE;
  transfer.data_type_id = MICROCAN_AUDIO_SPECTRUM_ID;
  transfer.inout_transfer_id = &transferId;
  transfer.priority = CANARD_TRANSFER_PRIORITY_LOW;
  transfer.payload = buffer.data();
  transfer.payload_len = size;
  transfer.canfd = fd;
  transfer.tao = not fd;
  const int16_t frames = canardBroadcastObj(&tx, &transfer);
  assert(frames > 0);
  received = 0U;
  expectedCount = count;
  unsigned frameCount = 0U;
  while (const auto *frame = canardPeekTxQueue(&tx)) {
    assert(canardHandleRxFrame(&rx, frame, (++frameCount) * 1000U) == CANARD_OK);
    canardPopTxQueue(&tx);
  }
  assert(received == 1U && frameCount == static_cast<unsigned>(frames));
  if (count == 10U) { assert(frames == (fd ? 1 : 7)); }
  if (count == 255U) { assert(frames == (fd ? 17 : 147)); }
}

int main()
{
  static_assert(SpectrumWire::maxBins == 255U);
  static_assert(SpectrumWire::maxBinsForPayload(0U) == 0U);
  static_assert(SpectrumWire::maxBinsForPayload(43U) == 10U);
  static_assert(SpectrumMessage<10U>::cxx_iface::MAX_SIZE == 43U);
  static_assert(MICROCAN_AUDIO_SPECTRUM_MAX_SIZE == 1023U);
  checkLevels();
  checkCodec<10U>();
  checkCodec<255U>();
  for (bool fd : {false, true}) {
    for (uint8_t count : {0U, 1U, 10U, 255U}) { checkTransport(fd, count); }
  }
  std::puts("Audio spectrum: uint16 scale, bounded encoder, 255-bin limit and CAN/CAN FD reassembly OK");
}
