#include "imavLightPattern.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

namespace {
struct Flash { uint32_t rise; uint32_t fall; };
struct Run {
  std::vector<uint32_t> events;
  std::vector<uint32_t> losses;
  std::vector<uint8_t> patterns;
};

std::vector<Flash> train(const std::vector<unsigned>& highs,
                         const std::vector<unsigned>& lows,
                         unsigned end = 18000U, unsigned start = 500U)
{
  std::vector<Flash> flashes;
  for (size_t index = 0U; start < end; ++index) {
    const size_t phase = index % highs.size();
    flashes.push_back({start, start + highs[phase]});
    start += highs[phase] + lows[phase];
  }
  return flashes;
}

Run replay(ImavLightPattern& detector, const std::vector<Flash>& flashes,
           uint32_t end, uint32_t offset = 0U, bool stuckOn = false,
           float onStrength = 0.95f)
{
  Run result;
  size_t flashIndex = 0U;
  for (uint32_t us = 0U; us < end * 1000U; us += 7200U) {
    const uint32_t now = us / 1000U;
    while (flashIndex < flashes.size() && flashes[flashIndex].fall <= now) {
      ++flashIndex;
    }
    const bool on = (flashIndex < flashes.size() &&
      now >= flashes[flashIndex].rise) ||
      (stuckOn && now >= flashes.back().fall);
    // Bounded score noise around already colour-qualified signal/background.
    const float score = on ? onStrength : 0.04f + 0.03f * std::sin(us * 0.001f);
    const auto event = detector.update(score, now + offset);
    if (event.flash) {
      result.events.push_back(now);
      result.patterns.push_back(detector.state().pulses);
      assert(onStrength >= 0.55f);
      bool observedFall = false;
      for (const Flash& flash : flashes) {
        if (now >= flash.fall && now - flash.fall <= 16U) {
          observedFall = true;
          break;
        }
      }
      // No synthesized flash during the long pause or a permanently on lamp.
      assert(observedFall);
    }
    if (event.lost) {
      result.losses.push_back(now);
    }
  }
  return result;
}

void repeatingPatterns()
{
  for (const auto& timings : std::vector<std::pair<std::vector<unsigned>,
                                                std::vector<unsigned>>>{
      {{270}, {420}},
      {{100, 100, 100}, {233, 217, 680}}}) {
    ImavLightPattern detector;
    const auto flashes = train(timings.first, timings.second);
    const auto result = replay(detector, flashes, 18000U);
    assert(!result.events.empty());
    assert(result.losses.empty());
    assert(detector.state().pulses == timings.first.size());
    // Exactly one event per real flash after acquisition, also across pauses.
    size_t expected = 0U;
    for (const auto& flash : flashes) {
      if (flash.fall + 16U < 18000U && flash.fall >= result.events.front() - 16U) {
        ++expected;
      }
    }
    assert(result.events.size() == expected);
  }
}

void measuredVideoTimings()
{
  // 60 fps video: first five visible triplets, before the pattern changes.
  const std::vector<Flash> flashes = {
    {5067, 5183}, {5400, 5500}, {5717, 5800},
    {6483, 6600}, {6817, 6900}, {7117, 7200},
    {7883, 8000}, {8217, 8317}, {8517, 8617},
    {9283, 9400}, {9617, 9717}, {9917, 10017},
    {10683, 10800}, {11017, 11117}, {11317, 11417},
  };
  ImavLightPattern detector;
  const auto result = replay(detector, flashes, 11600U);
  assert(result.events.size() == 6U);
  assert(result.losses.empty());
  for (uint8_t pulses : result.patterns) {
    assert(pulses == 3U);
  }
  assert(std::fabs(detector.state().cycleHz - 1.0f / 1.4f) < 0.02f);

  auto completeVideo = flashes;
  for (const auto& flash : std::vector<Flash>{
      {12100, 12367}, {12783, 13067}, {13467, 13750},
      {14167, 14433}, {14850, 15133}, {15550, 15817}}) {
    completeVideo.push_back(flash);
  }
  ImavLightPattern changing;
  const auto changed = replay(changing, completeVideo, 16000U);
  assert(changed.losses.size() == 1U);
  assert(changed.events.size() >= result.events.size() + 2U);
  assert(changing.state().pulses == 1U);
  assert(std::fabs(changing.state().cycleHz - 1.45f) < 0.03f);
}

void extinctionAndDiscontinuities()
{
  const auto flashes = train({100, 100, 100}, {220, 220, 680}, 10000U);
  for (const bool stuckOn : {false, true}) {
    ImavLightPattern detector;
    const auto result = replay(detector, flashes, 14000U, 0U, stuckOn);
    assert(!result.events.empty());
    assert(result.losses.size() == 1U);
    assert(result.losses.front() <= flashes.back().fall + 1500U);
    assert(!detector.detected());
  }
  ImavLightPattern detector;
  replay(detector, train({270}, {420}, 5000U), 5000U);
  assert(detector.detected());
  assert(detector.update(0.0f, 5200U).lost);
  assert(!detector.detected());
  // No partial history survives a dropout, explicit reset or invalid input.
  assert(!detector.update(0.9f, 5210U).flash);
  detector.reset();
  assert(!detector.detected());
  replay(detector, train({270}, {420}, 5000U), 5000U);
  assert(detector.update(std::numeric_limits<float>::quiet_NaN(), 5005U).lost);
}

void rejectNonPatterns()
{
  for (const float strength : {0.0f, 0.2f, 0.9f}) {
    ImavLightPattern detector;
    for (uint32_t now = 0U; now < 10000U; now += 7U) {
      const auto event = detector.update(strength, now);
      assert(!event.flash && !detector.detected());
    }
  }
  for (const auto& flashes : {
      train({15}, {318}), // isolated short disturbances
      train({800}, {200}), // excessively long illumination
      train({100}, {900}), // 1 Hz status light
      train({100}, {233}), // regular modes belong to the original detector
      train({100}, {400}),
      train({62}, {63}), // CREE stays a separate profile
      train({100}, {300}), // periodic, but neither 2 nor 3 Hz
      train({70, 130, 90, 180}, {200, 300, 240, 800}), // unsupported pattern
      std::vector<Flash>{{500, 600}, {833, 933}, {1166, 1266}}}) {
    ImavLightPattern detector;
    assert(replay(detector, flashes, 10000U).events.empty());
  }
  ImavLightPattern weak;
  assert(replay(weak, train({100}, {233}), 18000U, 0U, false, 0.4f).events.empty());
  std::vector<Flash> irregular;
  uint32_t seed = 123456U;
  uint32_t time = 500U;
  for (size_t i = 0U; i < 100U; ++i) {
    seed = seed * 1664525U + 1013904223U;
    const uint32_t high = 50U + seed % 400U;
    irregular.push_back({time, time + high});
    seed = seed * 1664525U + 1013904223U;
    time += high + 70U + seed % 1000U;
  }
  ImavLightPattern random;
  assert(replay(random, irregular, time + 1500U).events.empty());
}

void relearnAndWrap()
{
  auto flashes = train({100, 100, 100}, {220, 220, 680}, 10000U);
  const auto changed = train({150}, {350}, 20000U, 12000U);
  flashes.insert(flashes.end(), changed.begin(), changed.end());
  ImavLightPattern detector;
  const auto result = replay(detector, flashes, 20000U);
  assert(!result.events.empty() && !result.losses.empty());
  assert(!detector.detected()); // the spectral path takes over regular 2 Hz
  ImavLightPattern wrapped;
  const auto wrapResult = replay(wrapped, train({100, 100, 100}, {220, 220, 680}),
                                  18000U, UINT32_MAX - 6500U);
  assert(!wrapResult.events.empty());
  assert(wrapResult.losses.empty());
  assert(wrapped.state().pulses == 3U);

  auto missing = train({270}, {420});
  const uint32_t omitted = missing[20].rise;
  missing.erase(missing.begin() + 20);
  ImavLightPattern interrupted;
  const auto recovered = replay(interrupted, missing, 18000U);
  assert(recovered.losses.size() == 1U);
  assert(recovered.losses.front() >= omitted);
  assert(interrupted.detected());
}

void rgbQualification()
{
  const auto flashes = train({100, 100, 100}, {220, 220, 680}, 14000U);
  for (const auto& colour : std::vector<std::array<float, 3U>>{
      {1.0f, 0.10f, 0.03f}, {1.0f, 0.40f, 0.13f},
      {1.0f, 1.0f, 1.0f}, {0.1f, 1.0f, 0.1f}, {0.1f, 0.1f, 1.0f}}) {
    ImavLightPatternSignal signal;
    ImavLightPattern detector;
    unsigned events = 0U;
    for (uint32_t us = 0U; us < 15000000U; us += 7200U) {
      const uint32_t now = us / 1000U;
      std::array<float, 4U> rgb = {};
      for (size_t channel = 0U; channel < 3U; ++channel) {
        const uint32_t channelTime = now + static_cast<uint32_t>(channel * 2U);
        bool on = false;
        for (const auto& flash : flashes) {
          on = on || (channelTime >= flash.rise && channelTime < flash.fall);
        }
        // Sequential conversions, ambient drift and deterministic sensor noise.
        rgb[channel] = 20000.0f + 100.0f * std::sin(us * 0.0000003f) +
          20.0f * std::sin(us * 0.017f + channel) +
          (on ? 8000.0f * colour[channel] : 0.0f);
      }
      const auto event = detector.update(signal.update(rgb, now).score, now);
      if (event.flash) {
        ++events;
        bool observedFall = false;
        for (const auto& flash : flashes) {
          observedFall = observedFall ||
            (now >= flash.fall && now - flash.fall <= 20U);
        }
        assert(observedFall);
      }
    }
    if (colour[1] < 0.5f && colour[0] > 0.9f) {
      assert(events >= 15U);
    } else {
      assert(events == 0U);
    }
  }
  for (const bool tinyModulation : {false, true}) {
    ImavLightPatternSignal signal;
    ImavLightPattern detector;
    for (uint32_t now = 0U; now < 10000U; now += 7U) {
      std::array<float, 4U> rgb = {20000, 20000, 20000, 20000};
      if (tinyModulation) {
        rgb[0] += (now % 333U < 100U) ? 40.0f : 0.0f;
      }
      assert(!detector.update(signal.update(rgb, now).score, now).flash);
    }
  }
}
} // namespace

int main()
{
  repeatingPatterns();
  measuredVideoTimings();
  extinctionAndDiscontinuities();
  rejectNonPatterns();
  relearnAndWrap();
  rgbQualification();
  std::puts("Adaptive light: repeated patterns, measured video timings, real edges, "
            "extinction, invalid signals, relearning and clock wrap passed.");
}
