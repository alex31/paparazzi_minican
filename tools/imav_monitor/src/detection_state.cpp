#include "detection_state.hpp"

#include <algorithm>
#include <cmath>

DetectionState::DetectionState(std::chrono::milliseconds staleAfter)
  : staleAfter_(staleAfter) {}

bool DetectionState::update(std::string_view key,
                            float value,
                            TimePoint receivedAt) {
  if (!std::isfinite(value)) {
    return false;
  }

  TimedValue* destination = nullptr;
  if (key == "det") {
    destination = &detected_;
  } else if (key == "snr") {
    destination = &snr_;
  } else if (key == "lit") {
    destination = &light_;
  } else {
    return false;
  }

  destination->value = value;
  destination->receivedAt = receivedAt;
  destination->valid = true;
  lastNominalUpdate_ = receivedAt;
  return true;
}

DetectionSnapshot DetectionState::snapshot(TimePoint now) const {
  DetectionSnapshot result;
  const auto detected = freshValue(detected_, now);
  const auto light = freshValue(light_, now);

  if (detected.has_value()) {
    result.audioScore = clampScore(*detected);
  }
  if (light.has_value()) {
    result.lightScore = clampScore(*light);
  }
  if (result.audioScore.has_value() && result.lightScore.has_value()) {
    result.combinedScore = *result.audioScore * *result.lightScore;
  }
  result.snrDb = freshValue(snr_, now);
  return result;
}

std::optional<DetectionState::TimePoint>
DetectionState::lastNominalUpdate() const {
  return lastNominalUpdate_;
}

std::optional<float> DetectionState::freshValue(const TimedValue& value,
                                                TimePoint now) const {
  if (!value.valid || now < value.receivedAt ||
      now - value.receivedAt > staleAfter_) {
    return std::nullopt;
  }
  return value.value;
}

float DetectionState::clampScore(float value) {
  return std::clamp(value, 0.0F, 1.0F);
}
