"""Exercise production audio DSP, event scheduling and CAN SNR publication.

Run: python3 -m unittest discover -s tools/imav_monitor/tests -p test_audio_publication.py
Requires g++; the optional video replay also requires ffmpeg and the local clip.
No Python DSP dependencies. Hardware/RTOS/CAN are stubbed; optional telemetry is
omitted from the host build, while the operational SNR expiry code is retained.
"""
from array import array
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]


def method(source, name):
    start = source.index('void ImavRole::' + name + '(')
    opening = source.index('{', start)
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end] + '\n'


def build_replay(directory):
    source = (ROOT / 'COMMON/source/imavRole.cpp').read_text()
    constants = source[source.index('  constexpr size_t audioBufferDepth'):
                       source.index('  constexpr eventmask_t audioReadyEvent')]
    # Firmware metadata lives in the embedded parameter framework. Use its
    # default band here; coefficients and all scheduling constants stay intact.
    start = constants.index('  // Derive the configurable range')
    end = constants.index('  struct GoertzelBin')
    constants = constants[:start] + '''
constexpr uint16_t minimumAudioBandLowHz = 2000;
constexpr uint16_t maximumAudioBandLowHz = 2600;
constexpr uint16_t defaultAudioBandLowHz = 2000;
''' + constants[end:]
    structs = source[source.index('struct AudioChannelScore'):
                     source.index('static_assert(sizeof(adcsample_t)')]
    functions = source[source.index('  /** @brief Linear 0..1 knee'):
                       source.index('DeviceStatus ImavRole::subscribe')]
    functions = functions[:functions.rfind('}')]
    publication = method(source, 'publishMeasurements')
    publication = publication[:publication.index(
        '  if (not param_cget<"role.imav.debug.publish.optional">())')] + '}\n'
    preamble = r'''
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
using systime_t = uint32_t;
using sysinterval_t = uint32_t;
using gptcnt_t = uint32_t;
using adcsample_t = uint16_t;
using adcerror_t = uint32_t;
struct ADCConversionGroup {};
struct thread_t {};
struct ImavLightRange {};
static uint32_t hostTime;
static float hostAlpha = 1.0f;
static uint32_t chVTGetSystemTimeX() { return hostTime; }
static uint32_t chTimeDiffX(uint32_t a, uint32_t b) { return b - a; }
#define TIME_MS2I(ms) (static_cast<uint32_t>(ms) * 20U)
#define TIME_I2MS(t) (static_cast<uint32_t>(t) / 20U)
template<size_t N> struct ParameterName {
  char value[N];
  constexpr ParameterName(const char (&name)[N]) {
    std::copy_n(name, N, value);
  }
};
template<ParameterName Name> float param_cget() { return hostAlpha; }
constexpr int CANARD_TRANSFER_PRIORITY_LOW = 0;
struct uavcan_protocol_debug_KeyValue { float value; std::string key; };
namespace UAVCAN {
  void dsdlAssign(std::string& key, const char* value) { key = value; }
  struct Node {
    void sendBroadcast(const uavcan_protocol_debug_KeyValue& message, int) {
      printf("%u %s %.6f\n", hostTime, message.key.c_str(), message.value);
    }
  };
}
'''
    role = r'''
struct ImavRole {
  ImavAudioState* audio;
  UAVCAN::Node* m_node;
  void processAudioHalf(size_t offset, bool discontinuity);
  void publishAudioSnr();
  void publishMeasurements();
};
'''
    main = r'''
int main(int argc, char** argv) {
  if (argc < 2) return 2;
  if (argc > 2) hostAlpha = std::atof(argv[2]);
  ImavAudioState audio;
  UAVCAN::Node node;
  ImavRole role{&audio, &node};
  if (std::string(argv[1]) == "scores") {
    float score, frequency, snr;
    unsigned discontinuity;
    while (std::scanf("%u %f %f %f %u", &hostTime, &score,
                      &frequency, &snr, &discontinuity) == 5) {
      auto& d = audio.detector;
      d.blockScore = score;
      d.dominantFrequencyHz = frequency;
      d.channel.signalToNoiseDb = snr;
      if (updateAudioCadence(d, discontinuity != 0, hostTime)) {
        role.publishAudioSnr();
      }
      role.publishMeasurements();
    }
  } else {
    float samples[512];
    uint64_t total = 0;
    while (std::fread(samples, sizeof(float), 512, stdin) == 512) {
      total += 512;
      // The real worker discards the first DMA half after starting.
      if (total == 512) continue;
      for (size_t i = 0; i < 512; ++i) {
        audio.samples[i] = std::clamp(
          int(std::round(4096 + samples[i] * 3000)), 0, 8191);
      }
      hostTime = uint32_t(std::llround(total * 20000.0 / 23998));
      role.processAudioHalf(0, total == 1024);
    }
  }
}
'''
    cpp = directory / 'audio_replay.cpp'
    cpp.write_text(preamble + constants + structs + functions + role +
                   method(source, 'processAudioHalf') +
                   method(source, 'publishAudioSnr') + publication + main)
    binary = directory / 'audio_replay'
    subprocess.run(['g++', '-std=c++20', '-O2', '-Wall', '-Wextra',
                    str(cpp), '-o', str(binary)], check=True)
    return binary


class AudioPublicationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.directory.cleanup)
        cls.binary = build_replay(Path(cls.directory.name))

    def run_replay(self, rows, alpha=1):
        data = ''.join(f'{tick} {score} 2250 {snr} {gap}\n'
                       for tick, score, snr, gap in rows).encode()
        return self.run_input('scores', data, alpha)

    def run_input(self, mode, data, alpha=1):
        result = subprocess.run([str(self.binary), mode, str(alpha)],
                                input=data, capture_output=True, check=True)
        return [(int(tick), float(value)) for tick, key, value in
                (line.split() for line in result.stdout.decode().splitlines())
                if key == 'snr']

    def test_short_bursts_publish_only_at_their_end(self):
        for period in (333, 500):
            rows, ends = [], []
            was_on = False
            for ms in range(20, 2200, 20):
                on = ms >= 100 and (ms - 100) % period < 120
                if was_on and not on:
                    ends.append(ms * 20)
                rows.append((ms * 20, float(on), 18 if on else -60, 0))
                was_on = on
            messages = self.run_replay(rows)
            self.assertEqual([t for t, v in messages], ends)
            self.assertTrue(all(v == 18 for t, v in messages))

    def test_continuous_windows_follow_falling_levels_and_end(self):
        rows = [(ms * 20, float(100 <= ms < 1600),
                 24 if ms < 600 else 12, 0) for ms in range(20, 1900, 20)]
        messages = self.run_replay(rows)
        self.assertEqual([t // 20 for t, v in messages],
                         [320, 520, 720, 920, 1120, 1320, 1520, 1600])
        self.assertEqual([v for t, v in messages], [24, 24, 24, 12, 12, 12, 12, 12])
        smoothed = self.run_replay(rows, alpha=0.5)
        self.assertEqual([v for t, v in smoothed][3:6], [18, 15, 13.5])

    def test_end_on_periodic_deadline_publishes_once(self):
        rows = [(ms * 20, float(100 <= ms < 320), 18, 0)
                for ms in range(20, 700, 20)]
        self.assertEqual(self.run_replay(rows), [(320 * 20, 18)])

    def test_silence_noise_and_expiry(self):
        for score in (0.0, 0.2):
            messages = self.run_replay([(ms * 20, score, 20, 0)
                                       for ms in range(20, 3200, 20)])
            self.assertTrue(messages)
            self.assertTrue(all(v == 0 for t, v in messages))
        rows = [(ms * 20, float(100 <= ms < 1000), 18, 0)
                for ms in range(20, 3800, 20)]
        messages = self.run_replay(rows)
        zeros = [t // 20 for t, v in messages if v == 0]
        self.assertEqual(zeros, [2500, 3500])

    def test_discontinuity_or_stall_requires_rearming(self):
        for stall in (False, True):
            rows = []
            for ms in range(20, 1800, 20):
                if stall and 600 <= ms < 900:
                    continue
                score = float(100 <= ms < 1200 or ms >= 1300)
                rows.append((ms * 20, score, 18, int(not stall and ms == 600)))
            messages = self.run_replay(rows)
            self.assertEqual([t // 20 for t, v in messages if v > 0],
                             [320, 520, 1520, 1720])

    def test_tick_wrap_preserves_publication_period(self):
        base = (1 << 32) - 7000
        rows = [((base + ms * 20) % (1 << 32), float(ms >= 100), 18, 0)
                for ms in range(20, 1200, 20)]
        positive = [(t - base) % (1 << 32) // 20
                    for t, v in self.run_replay(rows) if v > 0]
        self.assertEqual(positive, [320, 520, 720, 920, 1120])

    def test_fyrefly2_recording(self):
        clip = ROOT / 'docs/VID_20260921_093815.mp4'
        if not clip.exists() or not shutil.which('ffmpeg'):
            self.skipTest('Local Fyrefly2 recording and ffmpeg required')
        audio = subprocess.run(['ffmpeg', '-v', 'error', '-i', str(clip),
                                '-vn', '-ar', '23998', '-ac', '1',
                                '-f', 'f32le', '-'],
                               capture_output=True, check=True).stdout
        messages = self.run_input('audio', audio)
        active = [(t / 20000, v) for t, v in messages if v > 0]
        self.assertEqual(len(active), 50)
        self.assertAlmostEqual(active[0][0], 6.25, delta=0.03)
        self.assertGreater(active[-1][0], 16.0)
        for (before, _), (after, _) in zip(active, active[1:]):
            self.assertAlmostEqual(after - before, 0.2, delta=0.022)
        self.assertTrue(all(v > 0 for t, v in messages if t / 20000 > 6.3))
        # Stopping the sound closes the remaining window, then expires normally.
        tail = array('f', [0.0] * (23998 * 3)).tobytes()
        with_silence = self.run_input('audio', audio + tail)
        final_active = [t / 20000 for t, v in with_silence if v > 0][-1]
        self.assertLess(final_active, 16.2)
        self.assertEqual(with_silence[-1][1], 0)


if __name__ == '__main__':
    unittest.main()
