#pragma once

#include <microcan.light.Measurement.h>
#include <algorithm>
#include <iterator>
#include <optional>

/** Publication policy, independent of the sensor, RTOS and CAN driver. */
class LightPublication {
public:
  using Message = microcan_light_Measurement;
  struct Settings {
    uint32_t intervalUs = 100'000U;
    uint32_t heartbeatUs = 1'000'000U;
    uint32_t deltaAbs = 1024U;
    float deltaRelPct = 5.0f;
    bool onChange = false;
  };

  void configure(Settings value) {
    *this = LightPublication{};
    settings = value;
  }

  void observe(const Message& sample) {
    latest = sample;
    if (not settings.onChange) {
      pending = sample;
      pending->reason = MICROCAN_LIGHT_MEASUREMENT_REASON_PERIODIC;
    } else {
      latchChange();
    }
  }

  [[nodiscard]] std::optional<uint64_t> deadline() const {
    const uint64_t earliest = attempted ? lastAttemptUs + settings.intervalUs : 0U;
    if (pending) {
      return earliest;
    }
    if (settings.onChange && latest && reference && settings.heartbeatUs != 0U) {
      return std::max(earliest, lastSuccessUs + settings.heartbeatUs);
    }
    return std::nullopt;
  }

  /** The caller records every attempt; CAN queue failure must not lose an event. */
  [[nodiscard]] const Message *due(uint64_t nowUs) {
    const auto next = deadline();
    if (not next || nowUs < *next) {
      return nullptr;
    }
    if (not pending) {
      pending = latest;
      pending->reason = MICROCAN_LIGHT_MEASUREMENT_REASON_HEARTBEAT;
    }
    return &*pending;
  }

  void complete(uint64_t nowUs, bool queued) {
    attempted = true;
    lastAttemptUs = nowUs;
    if (queued && pending) {
      reference = pending;
      pending.reset();
      lastSuccessUs = nowUs;
      if (settings.onChange) {
        // A flash may already have ended. Queue its return relative to the
        // sample just sent, retaining the actual acquisition timestamp.
        latchChange();
      }
    }
  }

private:
  [[nodiscard]] bool changed() const {
    for (size_t i = 0U; i < std::size(latest->rgbw); ++i) {
      const uint32_t value = latest->rgbw[i];
      const uint32_t baseline = reference->rgbw[i];
      const uint32_t delta = value > baseline ? value - baseline : baseline - value;
      // Test the integer floor exactly, even above float32's integer precision.
      if (delta > settings.deltaAbs &&
          static_cast<double>(delta) * 100.0 >
            static_cast<double>(baseline) * settings.deltaRelPct) {
        return true;
      }
    }
    return false;
  }

  void latchChange() {
    if (pending || not latest) {
      return;
    }
    if (not reference || latest->status != reference->status) {
      pending = latest;
      pending->reason = MICROCAN_LIGHT_MEASUREMENT_REASON_STATE;
    } else if ((latest->status & MICROCAN_LIGHT_MEASUREMENT_STATUS_VALID) && changed()) {
      pending = latest;
      pending->reason = MICROCAN_LIGHT_MEASUREMENT_REASON_CHANGE;
    }
  }

  Settings settings;
  std::optional<Message> latest;
  std::optional<Message> reference;
  std::optional<Message> pending;
  uint64_t lastAttemptUs = 0U;
  uint64_t lastSuccessUs = 0U;
  bool attempted = false;
};
