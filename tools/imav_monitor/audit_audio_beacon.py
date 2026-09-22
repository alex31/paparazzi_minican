#!/usr/bin/env python3
"""Reproduce the audio-only FireFly II audit (Python stdlib, g++, ffmpeg).

Uses the production-DSP host adapter from tests/test_audio_publication.py.
Reports observations, not hardware validation or assertions of correctness.
No firmware, device, or simulator files are modified.
"""

from array import array
import argparse
import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import random
import statistics
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
RATE = 23998


def decode(path, rate=RATE, filters=None):
    command = ['ffmpeg', '-v', 'error', '-i', str(path), '-vn', '-ac', '1']
    if filters:
        command += ['-af', filters]
    return subprocess.check_output(command + ['-ar', str(rate), '-f', 'f32le', '-'])


def float_bytes(values):
    samples = array('f', values)
    if sys.byteorder != 'little':
        samples.byteswap()
    return samples.tobytes()


def build(directory, source):
    # The older firmware uses another method name. Renaming that symbol keeps
    # the helper compatible without changing the old detection/publication.
    source = source.replace('publishAudioBurst', 'publishAudioSnr')
    snapshot = directory / 'snapshot'
    target = snapshot / 'COMMON/source/imavRole.cpp'
    target.parent.mkdir(parents=True)
    target.write_text(source)
    spec = importlib.util.spec_from_file_location(
        'audio_host_adapter', ROOT / 'tools/imav_monitor/tests/test_audio_publication.py')
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    helper.ROOT = snapshot
    binary = helper.build_replay(directory)
    cpp = directory / 'audio_replay.cpp'
    generated = cpp.read_text()
    # Two extra arguments select a persisted-band scenario and one lost-block
    # marker. Everything else, including ADC scaling, is the existing adapter.
    anchor = '  ImavRole role{&audio, &node};'
    assert generated.count(anchor) == 1
    generated = generated.replace(anchor, anchor + '''
  if (argc > 3) audio.detector.band = makeAudioBandConfiguration(std::atoi(argv[3]));
  const double gapTime = argc > 4 ? std::atof(argv[4]) : -1.0;
  bool gapInjected = false;
''')
    anchor = '      role.processAudioHalf(0, total == 1024);'
    assert generated.count(anchor) == 1
    generated = generated.replace(anchor, '''
      const bool gap = !gapInjected && gapTime >= 0 && hostTime / 20000.0 >= gapTime;
      gapInjected = gapInjected || gap;
      role.processAudioHalf(0, total == 1024 || gap);
''')
    cpp.write_text(generated)
    subprocess.run(['g++', '-std=c++20', '-O2', '-Wall', '-Wextra',
                    str(cpp), '-o', str(binary)], check=True)
    return binary


def replay(binary, data, output, name, band=2000, gap=-1):
    result = subprocess.run([str(binary), 'audio', '1', str(band), str(gap)],
                            input=data, capture_output=True, check=True)
    rows = [(int(tick) / 20000, key, float(value)) for tick, key, value in
            (line.split() for line in result.stdout.decode().splitlines())]
    with (output / (name + '.csv')).open('w') as file:
        writer = csv.writer(file)
        writer.writerow(['seconds', 'key', 'value'])
        writer.writerows(rows)
    positive = [(t, v) for t, key, v in rows if key == 'snr' and v > 0]
    zeros_after = [t for t, key, v in rows if key == 'snr' and v == 0
                   and positive and t > positive[-1][0]]
    summary = {
        'duration_s': len(data) / (4 * RATE),
        'band_low_hz': band,
        'discontinuity_at_s': gap if gap >= 0 else None,
        'positive_snr_messages': len(positive),
        'first_positive_s': positive[0][0] if positive else None,
        'last_positive_s': positive[-1][0] if positive else None,
        'positive_range_db': [min(v for t, v in positive), max(v for t, v in positive)]
        if positive else None,
        'first_zero_after_last_positive_s': zeros_after[0] if zeros_after else None,
    }
    if name.endswith('increasing_noise'):
        for start, end in [(2, 2.9), (4, 4.9)]:
            values = [v for t, v in positive if start <= t <= end]
            summary[f'mean_snr_{start}_to_{end}_s'] = statistics.mean(values) if values else None
    return summary


def frequency_summary(path, start, output, name):
    # Estimate the fundamental with 20 periods between interpolated upward
    # zero crossings; the bandpass suppresses harmonics. This is an offline
    # measurement, independent of the firmware's 50 Hz Goertzel grid.
    samples = array('f')
    samples.frombytes(decode(path, 48000, 'highpass=f=1600:p=2,lowpass=f=2800:p=2'))
    if sys.byteorder != 'little':
        samples.byteswap()
    crossings = [i - 1 - samples[i - 1] / (samples[i] - samples[i - 1])
                 for i in range(1, len(samples)) if samples[i - 1] < 0 <= samples[i]]
    frequency = [(crossings[i] / 48000,
                  20 * 48000 / (crossings[i + 10] - crossings[i - 10]))
                 for i in range(10, len(crossings) - 10, 10)
                 if crossings[i] / 48000 >= start]
    resets = [a[0] + (b[0] - a[0]) * (a[1] - 2150) / (a[1] - b[1])
              for a, b in zip(frequency, frequency[1:]) if a[1] >= 2150 > b[1]]
    periods = [(b - a) * 1000 for a, b in zip(resets, resets[1:])]
    with (output / (name + '_frequency.csv')).open('w') as file:
        writer = csv.writer(file)
        writer.writerow(['seconds_from_decoded_audio_start', 'frequency_hz'])
        writer.writerows(frequency)
    if not frequency or len(periods) < 2:
        return {'unavailable': 'Need at least three 2150 Hz downward crossings '
                f'after {start}s; analysis is specific to the reference recording.'}
    return {'frequency_min_hz': min(f for t, f in frequency),
            'frequency_max_hz': max(f for t, f in frequency),
            'periods_measured': len(periods),
            'period_mean_ms': statistics.mean(periods),
            'period_stddev_ms': statistics.stdev(periods)}


def main():
    sys.dont_write_bytecode = True
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--video', type=Path, default=ROOT / 'docs/VID_20260921_093815.mp4')
    parser.add_argument('--preview', type=Path, default=ROOT.parents[1] /
                        'DEVBOARDH7/devbh7_shell/assets/firefly2_preview.wav')
    parser.add_argument('--compare-ref', default='HEAD')
    parser.add_argument('--output', type=Path, default=Path('/tmp/imav-audio-audit'))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    data = decode(args.video)
    silence = float_bytes([0] * RATE)
    current = (ROOT / 'COMMON/source/imavRole.cpp').read_text()
    reference = subprocess.check_output(['git', '-C', str(ROOT), 'show',
        args.compare_ref + ':COMMON/source/imavRole.cpp'], text=True)
    provenance = {
        'current_source': hashlib.sha256(current.encode()).hexdigest(),
        'reference_source': hashlib.sha256(reference.encode()).hexdigest(),
        'host_adapter': hashlib.sha256((ROOT / 'tools/imav_monitor/tests/test_audio_publication.py')
                                      .read_bytes()).hexdigest(),
        'video': hashlib.sha256(args.video.read_bytes()).hexdigest(),
    }
    if args.preview.exists():
        provenance['simulator_preview'] = hashlib.sha256(args.preview.read_bytes()).hexdigest()
    report = {
        'reference_commit': subprocess.check_output(['git', '-C', str(ROOT),
                            'rev-parse', args.compare_ref], text=True).strip(),
        'sha256': provenance,
        'adapter': 'Production DSP; ADC=clamp(round(4096+3000*x),0,8191); '
                   '23,998 Hz; alpha=1; hardware/RTOS/CAN stubbed. '
                   'Lost-block case injects a discontinuity marker, not a DMA race.',
        'measurements': {'video': frequency_summary(args.video, 9, args.output, 'video')},
        'replays': {},
    }
    if not args.preview.exists():
        report['measurements']['simulator'] = {
            'skipped': f'Preview not found: {args.preview}; simulator was not replayed.'}
    with tempfile.TemporaryDirectory(prefix='imav-audio-audit-') as temporary:
        for label, source in [('current', current), ('reference', reference)]:
            directory = Path(temporary) / label
            directory.mkdir()
            binary = build(directory, source)
            cases = [('video', data, {}), ('video_then_silence', data + silence * 3, {})]
            if label == 'current':
                noise = random.Random(101)
                changing_noise = float_bytes(
                    noise.gauss(0, .005 if i < 3 * RATE else .12) +
                    (.2 * math.sin(2 * math.pi * 2250 * i / RATE) if i >= RATE else 0)
                    for i in range(5 * RATE))
                cases += [
                    ('video_start_at_7s', data[7 * RATE * 4:], {}),
                    ('video_discontinuity_at_9s', data, {'gap': 9}),
                    ('video_band_2600', data, {'band': 2600}),
                    ('increasing_noise', changing_noise, {}),
                    ('plain_2250hz_tone', silence + float_bytes(
                        0.2 * math.sin(2 * math.pi * 2250 * i / RATE)
                        for i in range(3 * RATE)) + silence * 2, {}),
                ]
                if args.preview.exists():
                    preview = decode(args.preview)
                    cases += [('simulator_already_on', preview, {}),
                              ('simulator_after_silence', silence + preview, {})]
                    report['measurements']['simulator'] = frequency_summary(
                        args.preview, .1, args.output, 'simulator')
            for name, samples, options in cases:
                key = label + '_' + name
                report['replays'][key] = replay(binary, samples, args.output, key, **options)
    rendered = json.dumps(report, indent=2) + '\n'
    (args.output / 'summary.json').write_text(rendered)
    print(rendered, end='')


if __name__ == '__main__':
    main()
