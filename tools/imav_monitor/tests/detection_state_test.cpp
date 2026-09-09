#include "detection_state.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <limits>

namespace {

bool nearlyEqual(float left, float right) {
  return std::abs(left - right) < 1.0e-6F;
}

} // namespace

int main() {
  using namespace std::chrono_literals;
  const DetectionState::TimePoint start{};
  DetectionState state(1000ms);

  const auto empty = state.snapshot(start);
  assert(!empty.audioScore.has_value());
  assert(!empty.lightScore.has_value());
  assert(!empty.combinedScore.has_value());
  assert(!empty.snrDb.has_value());

  assert(!state.update("aud", 0.8F, start));
  assert(!state.update("snr", std::numeric_limits<float>::quiet_NaN(),
                       start));
  assert(state.update("snr", 23.5F, start));
  assert(state.update("lit", 0.75F, start));

  const auto active = state.snapshot(start + 500ms);
  assert(active.audioScore.has_value());
  assert(active.lightScore.has_value());
  assert(active.combinedScore.has_value());
  assert(active.snrDb.has_value());
  assert(nearlyEqual(*active.audioScore, 1.0F));
  assert(nearlyEqual(*active.lightScore, 0.75F));
  assert(nearlyEqual(*active.combinedScore, 0.75F));
  assert(nearlyEqual(*active.snrDb, 23.5F));

  assert(state.snapshot(start + 1000ms).combinedScore.has_value());
  const auto stale = state.snapshot(start + 1001ms);
  assert(!stale.audioScore.has_value());
  assert(!stale.lightScore.has_value());
  assert(!stale.combinedScore.has_value());
  assert(!stale.snrDb.has_value());

  assert(state.update("snr", 0.0F, start + 2s));
  assert(state.update("lit", 2.0F, start + 2s));
  const auto clamped = state.snapshot(start + 2s);
  assert(nearlyEqual(*clamped.audioScore, 0.0F));
  assert(nearlyEqual(*clamped.lightScore, 1.0F));
  assert(nearlyEqual(*clamped.combinedScore, 0.0F));

  // Event-only firmware never sends det. Preserve the raw SNR on each burst
  // and use its positive/zero state for the binary audio corroboration.
  assert(state.update("snr", 0.1F, start + 2100ms));
  const auto weakBurst = state.snapshot(start + 2100ms);
  assert(nearlyEqual(*weakBurst.audioScore, 1.0F));
  assert(nearlyEqual(*weakBurst.snrDb, 0.1F));
  assert(nearlyEqual(*weakBurst.combinedScore, 1.0F));
  assert(state.update("snr", 42.0F, start + 2400ms));
  assert(nearlyEqual(*state.snapshot(start + 2400ms).snrDb, 42.0F));
  assert(state.update("snr", 0.0F, start + 2500ms));
  assert(nearlyEqual(*state.snapshot(start + 2500ms).audioScore, 0.0F));
  assert(nearlyEqual(*state.snapshot(start + 2500ms).combinedScore, 0.0F));

  // Updates and expiry of the two modalities are independent.
  assert(state.update("snr", 12.0F, start + 3s));
  const auto lightStale = state.snapshot(start + 3100ms);
  assert(lightStale.audioScore.has_value());
  assert(!lightStale.lightScore.has_value());
  assert(!lightStale.combinedScore.has_value());
  assert(state.update("lit", -0.1F, start + 4100ms));
  const auto audioStale = state.snapshot(start + 4100ms);
  assert(!audioStale.audioScore.has_value());
  assert(!audioStale.snrDb.has_value());
  assert(nearlyEqual(*audioStale.lightScore, 0.0F));
  assert(!audioStale.combinedScore.has_value());
  assert(!state.snapshot(start + 4s).lightScore.has_value());

  assert(!state.update("snr", std::numeric_limits<float>::infinity(),
                       start + 5s));
  assert(!state.update("det", 1.0F, start + 5s));
  assert(!state.update("aud", 1.0F, start + 5s));
  assert(state.lastNominalUpdate() == start + 4100ms);

  // The default timeout covers the firmware's 1.5 s expiry plus its 200 ms
  // publication cadence, and leaves margin around the idle 1 Hz heartbeats.
  DetectionState nominal;
  assert(nominal.update("snr", 23.5F, start));
  assert(nominal.update("lit", 0.9F, start));
  assert(nominal.snapshot(start + 1700ms).combinedScore.has_value());
  assert(nominal.update("snr", 0.0F, start + 1700ms));
  assert(nominal.update("lit", 0.0F, start + 1700ms));
  const auto idle = nominal.snapshot(start + 2800ms);
  assert(nearlyEqual(*idle.audioScore, 0.0F));
  assert(nearlyEqual(*idle.lightScore, 0.0F));
  assert(nearlyEqual(*idle.combinedScore, 0.0F));
  assert(nominal.snapshot(start + 3700ms).combinedScore.has_value());
  const auto disconnected = nominal.snapshot(start + 3701ms);
  assert(!disconnected.audioScore.has_value());
  assert(!disconnected.snrDb.has_value());
  assert(!disconnected.lightScore.has_value());
  assert(!disconnected.combinedScore.has_value());

  return 0;
}
