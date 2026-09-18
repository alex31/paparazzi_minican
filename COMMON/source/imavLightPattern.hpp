/**
 * @file imavLightPattern.hpp
 * @brief Learn repeating red-flash timings without predicting light events.
 */
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

/** @brief Normalize red flashes against a finite local illumination range. */
class ImavLightPatternSignal final {
public:
  struct State {
    float score = 0.0f;
    float redFraction = 0.0f;
    float relativeAc = 0.0f;
  };

  void reset()
  {
    // Invalidate the ring without creating a large temporary on the MCU stack.
    first = 0U;
    count = 0U;
    lastStoredMs = 0U;
    lastSampleMs = 0U;
    sampleSeen = false;
  }

  State update(const std::array<float, 4U>& scaled, uint32_t nowMs)
  {
    if (sampleSeen && (nowMs - lastSampleMs >= 100U)) {
      reset();
    }
    lastSampleMs = nowMs;
    sampleSeen = true;
    while (count != 0U && nowMs - samples[first].timeMs >= 1600U) {
      first = (first + 1U) % samples.size();
      --count;
    }
    const std::array<float, 3U> rgb = {scaled[0], scaled[1], scaled[2]};
    if ((count == 0U) || (nowMs - lastStoredMs >= 20U)) {
      if (count == samples.size()) {
        first = (first + 1U) % samples.size();
        --count;
      }
      samples[(first + count++) % samples.size()] = {nowMs, rgb};
      lastStoredMs = nowMs;
    }
    std::array<float, 3U> minimum = rgb;
    float low = contrast(rgb);
    float high = low;
    const float limit = std::numeric_limits<float>::max();
    std::array<float, 3U> lowest = {limit, limit, limit};
    std::array<float, 3U> highest = {-limit, -limit, -limit};
    for (size_t index = 0U; index < count; ++index) {
      const auto& sample = samples[(first + index) % samples.size()].rgb;
      for (size_t channel = 0U; channel < rgb.size(); ++channel) {
        minimum[channel] = std::min(minimum[channel], sample[channel]);
      }
      low = std::min(low, contrast(sample));
      high = std::max(high, contrast(sample));
      float lowCandidate = contrast(sample);
      float highCandidate = lowCandidate;
      for (size_t rank = 0U; rank < lowest.size(); ++rank) {
        if (lowCandidate < lowest[rank]) {
          std::swap(lowCandidate, lowest[rank]);
        }
        if (highCandidate > highest[rank]) {
          std::swap(highCandidate, highest[rank]);
        }
      }
    }
    if (count >= 10U) {
      // Two isolated extrema must not move the thresholds for a full window.
      low = lowest.back();
      high = highest.back();
    }
    const float span = high - low;
    const float red = rgb[0] - minimum[0];
    const float green = rgb[1] - minimum[1];
    const float blue = rgb[2] - minimum[2];
    const float redFraction = red / (red + green + blue + 1.0f);
    const float relative = span / (std::abs(minimum[0]) + 1024.0f);
    const float modulation = knee(relative, 0.005f, 0.025f) *
      knee(span, 64.0f, 512.0f);
    const float level = knee((contrast(rgb) - low) / (span + 1.0f),
                             0.20f, 0.60f);
    // White excursions alone stay below the 0.55 onset threshold. A modest
    // red excess can still be timed despite broadband ambient-light changes.
    const float colour = 0.4f + 0.6f * knee(redFraction, 0.42f, 0.68f);
    return {modulation * level * colour, redFraction, relative};
  }

private:
  struct Sample {
    uint32_t timeMs = 0U;
    std::array<float, 3U> rgb = {};
  };
  // Decimated range history: <= 81 samples, 1.6 s, no heap allocation.
  std::array<Sample, 81U> samples = {};
  size_t first = 0U;
  size_t count = 0U;
  uint32_t lastStoredMs = 0U;
  uint32_t lastSampleMs = 0U;
  bool sampleSeen = false;

  static float contrast(const std::array<float, 3U>& rgb)
  {
    return rgb[0] - 0.5f * (rgb[1] + rgb[2]);
  }
  static float knee(float value, float low, float high)
  {
    return std::clamp((value - low) / (high - low), 0.0f, 1.0f);
  }
};

/**
 * Input is a colour/contrast-qualified instantaneous light score.
 * Three repetitions establish a NEW video pattern: slow flashes or three
 * flashes plus a pause. Regular 2/3 Hz stays with the original spectral path.
 * Only observed, debounced falling edges publish a flash; a pause never
 * generates an event. Other periodic lights (including 1 Hz) are rejected.
 * The fixed history bounds both memory and the lifetime of acquired evidence.
 */
class ImavLightPattern final {
public:
  struct State {
    float score = 0.0f;
    float timingScore = 0.0f;
    float highMs = 0.0f;
    float lowMs = 0.0f;
    float cycleHz = 0.0f;
    uint8_t pulses = 0U;
  };

  struct Event {
    bool flash = false;
    bool lost = false;
  };

  static constexpr uint32_t maximumEventIntervalMs = 1800U;

  const State& state() const { return learned; }
  bool detected() const { return learned.pulses != 0U; }

  void reset()
  {
    *this = ImavLightPattern{};
  }

  Event update(float strength, uint32_t nowMs)
  {
    const bool wasDetected = detected();
    if ((sampleSeen && (nowMs - lastSampleMs >= 100U)) ||
        !std::isfinite(strength)) {
      reset();
    }
    sampleSeen = true;
    lastSampleMs = nowMs;
    if (!std::isfinite(strength)) {
      return {false, wasDetected};
    }

    // Expire a missing expected pulse promptly, including during silence.
    // Before acquisition use the largest supported gap, never unlimited time.
    const float allowedLow = detected()
      ? expectedLowMs + tolerance(expectedLowMs) : maximumLowMs;
    if (fallSeen && !on && (nowMs - lastFallMs > allowedLow + 25.0f)) {
      clearEvidence();
    }

    bool flash = false;
    if (!armed) {
      edgeSamples = strength <= 0.30f ? edgeSamples + 1U : 0U;
      if (edgeSamples >= 2U) {
        armed = true;
        edgeSamples = 0U;
      }
    } else if (!on) {
      if (strength >= 0.55f) {
        if (edgeSamples == 0U) {
          edgeMs = nowMs;
          peakStrength = strength;
        }
        peakStrength = std::max(peakStrength, strength);
        if (++edgeSamples >= 2U) {
          on = true;
          riseMs = edgeMs;
          edgeSamples = 0U;
        }
      } else {
        edgeSamples = 0U;
      }
    } else {
      peakStrength = std::max(peakStrength, strength);
      if (strength <= 0.30f) {
        if (edgeSamples == 0U) {
          edgeMs = nowMs;
        }
        if (++edgeSamples >= 2U) {
          on = false;
          edgeSamples = 0U;
          flash = finishPulse(edgeMs);
        }
      } else {
        edgeSamples = 0U;
      }
      const float allowedHigh = detected()
        ? expectedHighMs + tolerance(expectedHighMs) : maximumHighMs;
      if (on && (nowMs - riseMs > allowedHigh + 25.0f)) {
        // A stuck-on lamp cannot retain a lock or create repeated rises.
        clearEvidence();
        armed = false;
        on = false;
        edgeSamples = 0U;
      }
    }
    return {flash, wasDetected && !detected()};
  }

private:
  static constexpr size_t maximumPatternPulses = 3U;
  static constexpr size_t repetitions = 3U;
  static constexpr float minimumHighMs = 40.0f;
  static constexpr float maximumHighMs = 500.0f;
  static constexpr float minimumLowMs = 40.0f;
  static constexpr float maximumLowMs = 1200.0f;

  struct Pulse {
    float highMs = 0.0f;
    float lowMs = 0.0f;
    float strength = 0.0f;
  };
  std::array<Pulse, maximumPatternPulses * repetitions> history = {};
  size_t count = 0U;
  State learned;
  float expectedHighMs = 0.0f;
  float expectedLowMs = 0.0f;
  float peakStrength = 0.0f;
  uint32_t lastSampleMs = 0U;
  uint32_t riseMs = 0U;
  uint32_t lastFallMs = 0U;
  uint32_t edgeMs = 0U;
  uint8_t edgeSamples = 0U;
  bool sampleSeen = false;
  bool fallSeen = false;
  bool armed = false;
  bool on = false;

  static float tolerance(float durationMs)
  {
    // Includes sequential RGB conversion and roughly 7.2 ms group sampling.
    return std::max(30.0f, 0.20f * durationMs);
  }

  void clearEvidence()
  {
    count = 0U;
    fallSeen = false;
    learned = {};
    expectedHighMs = 0.0f;
    expectedLowMs = 0.0f;
  }

  bool finishPulse(uint32_t fallingMs)
  {
    const float high = static_cast<float>(fallingMs - riseMs);
    const float low = static_cast<float>(riseMs - lastFallMs);
    if ((high < minimumHighMs) || (high > maximumHighMs)) {
      clearEvidence();
      return false;
    }
    if (fallSeen && ((low < minimumLowMs) || (low > maximumLowMs))) {
      clearEvidence();
    }
    if (fallSeen) {
      if (count == history.size()) {
        for (size_t i = 1U; i < count; ++i) {
          history[i - 1U] = history[i];
        }
        --count;
      }
      history[count++] = {high, low, std::clamp(peakStrength, 0.0f, 1.0f)};
      findPattern();
    }
    lastFallMs = fallingMs;
    fallSeen = true;
    return detected();
  }

  void findPattern()
  {
    learned = {};
    // Learn timings inside the observed profile families; periodicity alone
    // does not establish that a light belongs to the beacon.
    for (size_t length = 1U; length <= maximumPatternPulses; ++length) {
      if (length == 2U) {
        continue;
      }
      const size_t needed = repetitions * length;
      if (count < needed) {
        break;
      }
      const size_t first = count - needed;
      float worstError = 0.0f;
      float minimumStrength = 1.0f;
      float cycleMs = 0.0f;
      float nextHigh = 0.0f;
      float nextLow = 0.0f;
      bool shortFlashes = true;
      size_t shortGaps = 0U;
      size_t longGaps = 0U;
      for (size_t phase = 0U; phase < length; ++phase) {
        float meanHigh = 0.0f;
        float meanLow = 0.0f;
        float minimumHigh = maximumHighMs;
        float maximumHigh = 0.0f;
        float minimumLow = maximumLowMs;
        float maximumLow = 0.0f;
        for (size_t repeat = 0U; repeat < repetitions; ++repeat) {
          const Pulse& pulse = history[first + repeat * length + phase];
          meanHigh += pulse.highMs / repetitions;
          meanLow += pulse.lowMs / repetitions;
          minimumHigh = std::min(minimumHigh, pulse.highMs);
          maximumHigh = std::max(maximumHigh, pulse.highMs);
          minimumLow = std::min(minimumLow, pulse.lowMs);
          maximumLow = std::max(maximumLow, pulse.lowMs);
          minimumStrength = std::min(minimumStrength, pulse.strength);
        }
        cycleMs += meanHigh + meanLow;
        shortFlashes = shortFlashes && (meanHigh >= 50.0f) &&
          (meanHigh <= 200.0f);
        shortGaps += (meanLow >= 150.0f) && (meanLow <= 300.0f);
        longGaps += (meanLow >= 500.0f) && (meanLow <= 900.0f);
        if (phase == 0U) {
          nextHigh = meanHigh;
          nextLow = meanLow;
        }
        // Bound the spread between repetitions, not just distance from their
        // mean (which would accept nearly twice the intended timing jitter).
        worstError = std::max({worstError,
          (maximumHigh - minimumHigh) / tolerance(meanHigh),
          (maximumLow - minimumLow) / tolerance(meanLow)});
      }
      const float frequencyHz = 1000.0f / cycleMs;
      const bool slowVideo = length == 1U &&
        (frequencyHz >= 1.3f) && (frequencyHz <= 1.6f) &&
        (nextHigh >= 220.0f) && (nextHigh <= 350.0f) &&
        (nextLow >= 300.0f) && (nextLow <= 550.0f);
      const bool triplet = length == 3U && shortFlashes &&
        (shortGaps == 2U) && (longGaps == 1U) &&
        (cycleMs >= 1100.0f) && (cycleMs <= 1800.0f);
      if ((worstError > 1.0f) || !(slowVideo || triplet)) {
        continue;
      }
      const float timing = 1.0f - 0.35f * worstError * worstError;
      const float score = minimumStrength * timing;
      if (score < 0.55f) {
        continue;
      }
      learned = {score, timing, history[count - 1U].highMs,
                 history[count - 1U].lowMs, frequencyHz,
                 static_cast<uint8_t>(length)};
      expectedHighMs = nextHigh;
      expectedLowMs = nextLow;
      return;
    }
  }
};
