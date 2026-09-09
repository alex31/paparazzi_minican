#pragma once

#include <chrono>
#include <optional>
#include <string_view>

struct DetectionSnapshot {
  // Binary presence from a fresh positive snr; nullopt means no fresh data.
  std::optional<float> audioScore;
  std::optional<float> lightScore;
  std::optional<float> combinedScore;
  std::optional<float> snrDb;
};

/**
 * Holds nominal IMAV snr/lit values and applies their freshness policy.
 *
 * The combined score is deliberately conservative: audioScore * lightScore.
 * It therefore represents corroboration by both sensors rather than an OR.
 */
class DetectionState {
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  explicit DetectionState(
    std::chrono::milliseconds staleAfter = std::chrono::milliseconds(2000));

  /** Return true when key is nominal and the finite value was accepted. */
  bool update(std::string_view key, float value, TimePoint receivedAt);

  DetectionSnapshot snapshot(TimePoint now) const;
  std::optional<TimePoint> lastNominalUpdate() const;

private:
  struct TimedValue {
    float value = 0.0F;
    TimePoint receivedAt{};
    bool valid = false;
  };

  std::optional<float> freshValue(const TimedValue& value,
                                  TimePoint now) const;
  static float clampScore(float value);

  std::chrono::milliseconds staleAfter_;
  TimedValue snr_;
  TimedValue light_;
  std::optional<TimePoint> lastNominalUpdate_;
};
