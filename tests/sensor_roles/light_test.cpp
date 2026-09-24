#include "lightPublication.hpp"
#include "opt4060Timing.hpp"
#include <cassert>
#include <cstdio>
#include <limits>

using Message = LightPublication::Message;
constexpr uint8_t valid = MICROCAN_LIGHT_MEASUREMENT_STATUS_VALID;
constexpr uint8_t error = MICROCAN_LIGHT_MEASUREMENT_STATUS_ERROR;
constexpr uint8_t saturated = MICROCAN_LIGHT_MEASUREMENT_STATUS_SATURATED;

static Message sample(uint64_t stamp, uint32_t value, uint8_t status = valid)
{
  return {.timestamp_us = stamp, .conversion_time_us = 1800U,
          .sensor_id = 2U, .status = status, .reason = 0U,
          .rgbw = {value, value, value, value}};
}

static void checkTiming()
{
  using namespace Opt4060Timing;
  const auto automatic = configure(10U, 0U, 400U);
  assert(automatic && not automatic->onChange && automatic->conversionUs == 12'700U);
  const auto equal = configure(10U, 10U, 400U);
  assert(equal && not equal->onChange && equal->conversionUs == automatic->conversionUs);
  const auto event = configure(10U, 100U, 400U);
  assert(event && event->onChange && event->conversionUs == 1800U);
  assert(event->triggerConfig() == 0xB098U);
  assert(configure(10U, 100U, 100U)->conversionUs == 1000U);
  assert(configure(1U, 0U, 400U)->conversionUs == 200'000U);
  assert(configure(10U, 9U, 400U).error() == Error::ScanBelowPublish);
  assert(configure(0U, 0U, 400U).error() == Error::Frequency);
  assert(not configure(201U, 0U, 400U));
  assert(not configure(10U, 201U, 400U));
  assert(not configure(10U, 0U, 1000U));
  assert(configure(10U, 200U, 100U).error() == Error::ConversionBudget);
  assert(configure(10U, 200U, 400U)->conversionUs == 600U);
  for (const uint32_t bus : {100U, 400U}) {
    for (uint32_t hz = 1U; hz <= 200U; ++hz) {
      const auto setting = configure(hz, 0U, bus);
      if (not setting) { continue; }
      const uint32_t margin = bus == 100U ? 4000U : 2000U;
      assert(setting->scanIntervalUs * hz >= 1'000'000U);
      assert(setting->scanIntervalUs == setting->publishIntervalUs);
      assert(setting->conversionUs * 4U + margin <= 1'000'000U / hz);
      const auto next = setting->conversionIndex + 1U;
      assert(next == conversionTimesUs.size() ||
             conversionTimesUs[next] * 4U + margin > 1'000'000U / hz);
    }
  }
}

static LightPublication eventPolicy(uint32_t heartbeat = 1'000'000U)
{
  LightPublication policy;
  policy.configure({.intervalUs = 100'000U, .heartbeatUs = heartbeat,
                    .deltaAbs = 10U, .deltaRelPct = 5.0f, .onChange = true});
  assert(not policy.due(0U));
  policy.observe(sample(0U, 1000U));
  assert(policy.due(0U)->reason == MICROCAN_LIGHT_MEASUREMENT_REASON_STATE);
  policy.complete(0U, true);
  return policy;
}

static void checkEvents()
{
  auto policy = eventPolicy();
  // Slow drift accumulates relative to the transmitted baseline.
  for (uint32_t step = 1U; step <= 5U; ++step) {
    policy.observe(sample(step * 10'000U, 1000U + 10U * step));
    assert(not policy.due(step * 10'000U));
  }
  auto change = sample(60'000U, 1000U);
  change.rgbw[2] = 1051U; // Any one channel, strict > threshold.
  policy.observe(change);
  assert(not policy.due(99'999U));
  // Return before CAN publication must not erase the captured event.
  policy.observe(sample(70'000U, 1000U));
  const auto *pending = policy.due(100'000U);
  assert(pending && pending->timestamp_us == 60'000U && pending->rgbw[2] == 1051U);
  assert(pending->reason == MICROCAN_LIGHT_MEASUREMENT_REASON_CHANGE);
  policy.complete(100'000U, false); // Queue full: preserve event and baseline.
  assert(not policy.due(199'999U));
  assert(policy.due(200'000U)->timestamp_us == 60'000U);
  policy.complete(200'000U, true);
  // 51 < 5% of 1051: the asymmetric relative deadband intentionally ignores
  // this small return. A larger flash below checks that the return is queued.
  assert(not policy.due(300'000U));

  policy = eventPolicy();
  policy.observe(sample(10'000U, 2000U));
  policy.observe(sample(20'000U, 1000U));
  assert(policy.due(100'000U)->rgbw[0] == 2000U);
  policy.complete(100'000U, true);
  assert(not policy.due(199'999U));
  assert(policy.due(200'000U)->timestamp_us == 20'000U);
  assert(policy.due(200'000U)->rgbw[0] == 1000U);
  policy.complete(200'000U, true);
  assert(not policy.due(300'000U));

  // Near darkness, the absolute floor prevents relative-noise events.
  policy.configure({.intervalUs = 1U, .heartbeatUs = 0U,
                    .deltaAbs = 10U, .deltaRelPct = 5.0f, .onChange = true});
  policy.observe(sample(0U, 0U));
  assert(policy.due(0U));
  policy.complete(0U, true);
  policy.observe(sample(1U, 10U));
  assert(not policy.due(1U));
  policy.observe(sample(2U, 11U));
  assert(policy.due(2U));

  // Large integers must not overflow, or lose a one-count delta to float32.
  policy.configure({.intervalUs = 1U, .heartbeatUs = 0U,
                    .deltaAbs = 0U, .deltaRelPct = 0.0f, .onChange = true});
  policy.observe(sample(0U, std::numeric_limits<uint32_t>::max()));
  assert(policy.due(0U));
  policy.complete(0U, true);
  policy.observe(sample(1U, std::numeric_limits<uint32_t>::max() - 1U));
  assert(policy.due(1U));
}

static void checkHeartbeatAndState()
{
  auto policy = eventPolicy();
  policy.observe(sample(900'000U, 1001U));
  assert(not policy.due(999'999U));
  assert(policy.due(1'000'000U)->reason == MICROCAN_LIGHT_MEASUREMENT_REASON_HEARTBEAT);
  assert(policy.due(1'000'000U)->timestamp_us == 900'000U);
  policy.complete(1'000'000U, true);
  assert(not policy.due(1'999'999U));

  policy = eventPolicy(10U); // Heartbeat also obeys the publication limit.
  assert(not policy.due(99'999U));
  assert(policy.due(100'000U));
  policy = eventPolicy(0U);
  assert(not policy.due(60'000'000U));
  policy.observe(sample(60'000'000U, 0U, error));
  assert(policy.due(60'000'000U)->reason == MICROCAN_LIGHT_MEASUREMENT_REASON_STATE);
  policy.complete(60'000'000U, true);
  policy.observe(sample(60'010'000U, 1000U));
  assert(not policy.due(60'099'999U));
  assert(policy.due(60'100'000U)->status == valid);
  policy.complete(60'100'000U, true);
  policy.observe(sample(60'110'000U, 0x3FFFFC0U, saturated));
  assert(policy.due(60'200'000U)->status == saturated);
  policy.complete(60'200'000U, true);
  policy.observe(sample(60'210'000U, 1U, saturated));
  assert(not policy.due(60'300'000U)); // Unusable counts never trigger deltas.
}

static void checkPeriodicAndLongUptime()
{
  LightPublication policy;
  policy.configure({.intervalUs = 100'000U, .onChange = false});
  constexpr uint64_t start = 0xFFFFFFFFULL * 50U; // Across 32-bit RTOS tick wrap.
  policy.observe(sample(start, 1000U));
  assert(policy.due(start)->reason == MICROCAN_LIGHT_MEASUREMENT_REASON_PERIODIC);
  policy.complete(start, true);
  assert(not policy.due(start + 200'000U)); // No duplicate acquisition.
  policy.observe(sample(start + 1U, 1000U));
  assert(not policy.due(start + 99'999U));
  policy.observe(sample(start + 50'000U, 1001U));
  assert(policy.due(start + 100'000U)->rgbw[0] == 1001U);
  policy.complete(start + 100'000U, false);
  policy.observe(sample(start + 150'000U, 1002U));
  assert(not policy.due(start + 199'999U));
  assert(policy.due(start + 200'000U)->rgbw[0] == 1002U);
  policy.complete(start + 200'000U, true);
  assert(not policy.due(start + 300'000U));
}

static unsigned received;
static Message expected;
static void onReceive(CanardInstance*, CanardRxTransfer *transfer)
{
  Message decoded = {};
  assert(not microcan_light_Measurement_decode(transfer, &decoded));
  assert(decoded.timestamp_us == expected.timestamp_us);
  assert(decoded.conversion_time_us == expected.conversion_time_us);
  assert(decoded.sensor_id == expected.sensor_id);
  assert(decoded.status == expected.status && decoded.reason == expected.reason);
  assert(std::ranges::equal(decoded.rgbw, expected.rgbw));
  ++received;
}

static bool shouldAccept(const CanardInstance*, uint64_t *signature, uint16_t id,
                         CanardTransferType type, uint8_t)
{
  if (id != MICROCAN_LIGHT_MEASUREMENT_ID || type != CanardTransferTypeBroadcast) {
    return false;
  }
  *signature = MICROCAN_LIGHT_MEASUREMENT_SIGNATURE;
  return true;
}

static void checkTransport(bool fd)
{
  alignas(16) std::array<uint8_t, 4096> txPool{}, rxPool{};
  CanardInstance tx, rx;
  canardInit(&tx, txPool.data(), txPool.size(), onReceive, shouldAccept, nullptr);
  canardInit(&rx, rxPool.data(), rxPool.size(), onReceive, shouldAccept, nullptr);
  canardSetLocalNodeID(&tx, 42U);
  canardSetLocalNodeID(&rx, 43U);
  expected = sample(0x123456789ABCULL, 0x3FFFFC0U);
  expected.rgbw[1] = 0U;
  expected.rgbw[2] = 0x12345678U;
  expected.reason = MICROCAN_LIGHT_MEASUREMENT_REASON_CHANGE;
  std::array<uint8_t, MICROCAN_LIGHT_MEASUREMENT_MAX_SIZE> bytes{};
  const auto size = microcan_light_Measurement_encode(&expected, bytes.data(), not fd);
  assert(size == 31U);
  uint8_t transferId = 0U;
  CanardTxTransfer transfer = {};
  transfer.transfer_type = CanardTransferTypeBroadcast;
  transfer.data_type_signature = MICROCAN_LIGHT_MEASUREMENT_SIGNATURE;
  transfer.data_type_id = MICROCAN_LIGHT_MEASUREMENT_ID;
  transfer.inout_transfer_id = &transferId;
  transfer.priority = CANARD_TRANSFER_PRIORITY_LOW;
  transfer.payload = bytes.data();
  transfer.payload_len = size;
  transfer.canfd = fd;
  transfer.tao = not fd;
  assert(canardBroadcastObj(&tx, &transfer) == (fd ? 1 : 5));
  received = 0U;
  unsigned frameCount = 0U;
  while (const auto *frame = canardPeekTxQueue(&tx)) {
    assert(canardHandleRxFrame(&rx, frame, (++frameCount) * 1000U) == CANARD_OK);
    canardPopTxQueue(&tx);
  }
  assert(received == 1U);
  CanardRxTransfer truncated = {};
  truncated.payload_head = bytes.data();
  truncated.payload_len = size - 1U;
  truncated.tao = not fd;
  Message decoded = {};
  assert(microcan_light_Measurement_decode(&truncated, &decoded));
}

int main()
{
  checkTiming();
  checkEvents();
  checkHeartbeatAndState();
  checkPeriodicAndLongUptime();
  checkTransport(false);
  checkTransport(true);
  std::puts("Light: timing, drift/flash/return, backpressure, state/heartbeat and CAN/CAN FD OK");
}
