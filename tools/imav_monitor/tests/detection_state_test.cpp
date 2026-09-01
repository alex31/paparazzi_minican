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
  assert(!state.update("det", std::numeric_limits<float>::quiet_NaN(),
                       start));
  assert(state.update("det", 1.2F, start));
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

  assert(state.update("det", -1.0F, start + 2s));
  assert(state.update("lit", 2.0F, start + 2s));
  const auto clamped = state.snapshot(start + 2s);
  assert(nearlyEqual(*clamped.audioScore, 0.0F));
  assert(nearlyEqual(*clamped.lightScore, 1.0F));
  assert(nearlyEqual(*clamped.combinedScore, 0.0F));

  return 0;
}
