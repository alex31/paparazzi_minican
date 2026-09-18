"""Run the firmware light processing/publication methods on a host.

Only RTOS time/locks, CAN output and sensor I/O are stubbed. The constructor,
DSP, baseline, selection, timeout and publication methods come from production
sources, so comparing enabled/disabled exercises the actual integration.
"""
from pathlib import Path
import json
import io
import subprocess
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools/imav_monitor"))
from offline_light_detector import load_samples


def method(source, name):
    start = source.index("ImavLightRange::" + name + "(")
    start = source.rfind("\n", 0, start) + 1
    # The adaptive method has its return type on the same line.
    opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end] + "\n"


def build_replay(directory, source=None, header=None):
    source = source or (ROOT / "COMMON/source/imavLightRange.cpp").read_text()
    header = header or (ROOT / "COMMON/source/imavLightRange.hpp").read_text()
    declarations = header[header.index("struct ImavLightRangeSnapshot"):]
    declarations = declarations.replace("private:", "public:")
    constants = source[source.index("  constexpr float lightFastBinSpacingHz"):
                       source.index("  static_assert(optConfigurationPowerDown")]
    constructor = source[source.index("ImavLightRange::ImavLightRange("):
                         source.index('extern "C" uint8_t *imav_vl53l4cx_work_buffer')]
    names = ["snapshot", "publishLightState", "publishLightScore",
             "serviceLightScore", "resetLightFastSpectrum", "lightSpectralSignal",
             "accumulateLightFastSample", "scoreLightFastPattern",
             "evaluateLightFastSpectrum", "updateSynchronizedLightEvent",
             "updateLightFastSpectrum", "processLightMeasurement"]
    adaptive = "adaptivePatternEnabled" in header
    if adaptive:
        names += ["publishSpectralLightScore", "updateAdaptiveLightPattern"]
    preamble = r'''
#include "imavLightPattern.hpp"
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <vector>
using systime_t = uint32_t;
using sysinterval_t = uint32_t;
using msg_t = int;
struct VL53L4CX_Object_t {};
static uint32_t hostTime;
static uint32_t chVTGetSystemTimeX() { return hostTime; }
static uint32_t chTimeDiffX(uint32_t a, uint32_t b) { return b - a; }
static void chSysLock() {}
static void chSysUnlock() {}
#define chDbgAssert(a,b) ((void)0)
#define TIME_MS2I(ms) (static_cast<uint32_t>(ms) * 20U)
#define TIME_S2I(s) (static_cast<uint32_t>(s) * 20000U)
#define TIME_I2MS(t) (static_cast<uint32_t>(t) / 20U)
constexpr int CANARD_TRANSFER_PRIORITY_LOW = 0;
constexpr size_t optChannelCount = 4U;
struct uavcan_protocol_debug_KeyValue { float value = 0; int key = 0; };
namespace UAVCAN {
  static void dsdlAssign(int&, const char*) {}
  struct Node {
    std::vector<float> values;
    void sendBroadcast(const uavcan_protocol_debug_KeyValue& message, int) {
      values.push_back(message.value);
    }
  };
}
static float knee(float value, float low, float high) {
  return std::clamp((value - low) / (high - low), 0.0f, 1.0f);
}
'''
    main = r'''
void ImavLightRange::publishLightDebugSample(bool) {}
int main(int argc, char** argv) {
  assert(argc == 4);
  UAVCAN::Node node;
  ImavLightRange light(node, 0x44, 200, false, std::atoi(argv[2]) != 0,
                      100, 233, 400, std::atoi(argv[3]) != 0 ADAPTIVE_ARGUMENT);
  std::array<uint32_t, 4U> rgb;
  while (std::scanf("%u %u %u %u %u", &hostTime, &rgb[0], &rgb[1],
                    &rgb[2], &rgb[3]) == 5) {
    node.values.clear();
    light.serviceLightScore(hostTime);
    light.processLightMeasurement(rgb, false, hostTime);
    auto state = light.snapshot();
    assert(node.values.size() <= 2U);
    std::printf("%u %.9g %u %u %u %zu %.9g %.9g\n", hostTime,
                light.lightFastScore, light.lightFastDetected,
                VIDEO_DETECTED, state.lightPattern, node.values.size(),
                node.values.empty() ? -1.0f : node.values[0],
                node.values.size() < 2U ? -1.0f : node.values[1]);
  }
}
'''
    main = main.replace("ADAPTIVE_ARGUMENT", ", std::atoi(argv[1]) != 0" if adaptive else "")
    main = main.replace("VIDEO_DETECTED", "light.adaptiveLightPattern.detected()" if adaptive else "false")
    cpp = directory / "light_replay.cpp"
    cpp.write_text(preamble + constants + declarations + constructor +
                   "\n".join(method(source, name) for name in names) + main)
    binary = directory / "light_replay"
    subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-I", str(ROOT / "COMMON/source"), str(cpp), "-o", str(binary)],
                   check=True, capture_output=True, text=True)
    return binary


def run_replay(binary, samples, adaptive=False, beginning=False, cree=False):
    raw = samples[:, 2:6].copy()
    raw[:, 0] /= 2.4
    raw[:, 2] /= 1.3
    records = np.column_stack((np.rint(samples[:, 1] * 20000), np.rint(raw)))
    data = io.StringIO()
    np.savetxt(data, records.astype(np.uint32), fmt="%d")
    result = subprocess.run([str(binary), str(int(adaptive)), str(int(beginning)),
                             str(int(cree))], input=data.getvalue(), text=True,
                            capture_output=True, check=True)
    return np.loadtxt(io.StringIO(result.stdout))


def rgb_train(highs, lows, duration=16.0, amplitude=10000.0):
    time = np.arange(0.0, duration, 0.0072)
    rgb = np.full((len(time), 3), 20000.0)
    channel_time = time[:, None] + np.array([0.0, 0.0018, 0.0036])
    start = 0.5
    falls = []
    index = 0
    while start < duration:
        phase = index % len(highs)
        end = start + highs[phase]
        rgb += ((channel_time >= start) & (channel_time < end)) * amplitude * np.array([1.0, 0.1, 0.03])
        falls.append(end)
        start = end + lows[phase]
        index += 1
    return np.column_stack((time, time, rgb, np.mean(rgb, axis=1))), np.asarray(falls)


class LightExtensionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix="imav-light-extension-")
        try:
            cls.binary = build_replay(Path(cls.temporary.name))
        except subprocess.CalledProcessError as error:
            raise RuntimeError(error.stderr) from error

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def test_original_events_and_scores_unchanged(self):
        for high, low in [(0.100, 0.233), (0.100, 0.400)]:
            for beginning in [False, True]:
                for amplitude in [1500.0, 10000.0]:
                    with self.subTest(high=high, low=low, beginning=beginning, amplitude=amplitude):
                        samples, _ = rgb_train([high], [low], amplitude=amplitude)
                        before = run_replay(self.binary, samples, beginning=beginning)
                        after = run_replay(self.binary, samples, adaptive=True, beginning=beginning)
                        np.testing.assert_array_equal(after, before)

    def test_cree_remains_separate(self):
        samples, _ = rgb_train([0.062], [0.063])
        before = run_replay(self.binary, samples, cree=True)
        after = run_replay(self.binary, samples, adaptive=True, cree=True)
        np.testing.assert_array_equal(after, before)
        self.assertTrue(np.any(after[:, 6] > 0))

    def test_video_profiles_add_observed_events_and_hold_pauses(self):
        for highs, lows in [([0.1] * 3, [0.233, 0.217, 0.680]), ([0.270], [0.420])]:
            with self.subTest(highs=highs):
                samples, falls = rgb_train(highs, lows)
                baseline = run_replay(self.binary, samples)
                extended = run_replay(self.binary, samples, adaptive=True)
                # The spectral path is still updated identically on every sample.
                np.testing.assert_array_equal(extended[:, 1:3], baseline[:, 1:3])
                locked = np.flatnonzero(extended[:, 3] == 1)
                self.assertGreater(len(locked), 100)
                active = extended[locked[0]:]
                self.assertTrue(np.all(active[:, 3] == 1))
                # No zero from the legacy path may cancel the new pattern.
                self.assertFalse(np.any(active[:, 6:8] == 0))
                events = active[active[:, 6] > 0, 0] / 20000
                self.assertGreater(len(events), 12)
                errors = [np.min(np.abs(falls - event)) for event in events]
                self.assertLess(max(errors), 0.025)
                for fall in falls:
                    if events[0] <= fall <= events[-1] - 0.025:
                        self.assertEqual(sum(np.abs(events - fall) < 0.025), 1)

    def test_existing_rgbw_captures_unchanged(self):
        monitor = ROOT / "tools/imav_monitor"
        manifest = json.loads((monitor / "light_capture_segments.json").read_text())
        files = [monitor / "captures" / name for name in manifest]
        if not all(path.exists() for path in files):
            self.skipTest("Local RGBW captures are not distributed with the repository")
        for path in files:
            samples = load_samples(path)
            for beginning in [False, True]:
                with self.subTest(capture=path.name, beginning=beginning):
                    baseline = run_replay(self.binary, samples, beginning=beginning)
                    extended = run_replay(self.binary, samples, adaptive=True, beginning=beginning)
                    np.testing.assert_array_equal(extended, baseline)

    def test_return_to_standard_and_extinction(self):
        video, _ = rgb_train([0.1] * 3, [0.233, 0.217, 0.680], duration=12)
        regular, _ = rgb_train([0.1], [0.233], duration=18)
        regular[:, :2] += 12.0
        samples = np.vstack((video, regular))
        samples[samples[:, 0] >= 26.0, 2:] = 20000.0
        baseline = run_replay(self.binary, samples)
        extended = run_replay(self.binary, samples, adaptive=True)
        self.assertTrue(np.any(extended[:, 3] == 1))
        # Once the signal becomes regular, original timing/score/publication
        # resumes, with no residual adaptive hold or additional learning delay.
        np.testing.assert_array_equal(extended[samples[:, 0] >= 15],
                                      baseline[samples[:, 0] >= 15])
        self.assertFalse(np.any(extended[samples[:, 0] >= 27, 6:8] > 0))


if __name__ == "__main__":
    unittest.main()
