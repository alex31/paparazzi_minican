#!/usr/bin/env python3
"""Replay a finite-memory IMAV optical detector from lossless CAN captures."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
from typing import Iterable

import numpy as np


SCRIPT_DIRECTORY = Path(__file__).resolve().parent
DEFAULT_MANIFEST = SCRIPT_DIRECTORY / "light_capture_segments.json"
DEFAULT_CAPTURE_DIRECTORY = SCRIPT_DIRECTORY / "captures"
TIMESTAMP_MODULUS_US = 1 << 24


@dataclass(frozen=True)
class Pattern:
    identifier: int
    name: str
    high_ms: float
    low_ms: float

    @property
    def frequency_hz(self) -> float:
        return 1000.0 / (self.high_ms + self.low_ms)

    @property
    def expected_harmonic_ratio(self) -> float:
        duty_cycle = self.high_ms / (self.high_ms + self.low_ms)
        return abs(math.cos(math.pi * duty_cycle))


@dataclass(frozen=True)
class Detection:
    elapsed_s: float
    score: float
    pattern: int
    frequency_hz: float
    coherence: float
    harmonic_ratio: float
    phase_alignment: float
    red_fraction: float
    harmonic_shape: float


STEADY_PATTERN = Pattern(2, "steady", 100.0, 233.0)
BEGINNING_PATTERN = Pattern(1, "beginning", 100.0, 400.0)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Replay the one-second fundamental/H2 optical detector from "
            "lrd/lgn/lbl/lwh/lct/ltu CAN records."
        )
    )
    parser.add_argument("captures", nargs="*", type=Path,
                        help="capture CSV files; the manifest is used by default")
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--window-ms", type=float, default=1000.0)
    parser.add_argument("--step-ms", type=float, default=50.0)
    parser.add_argument("--high-ms", type=float, default=100.0)
    parser.add_argument("--steady-low-ms", type=float, default=233.0)
    parser.add_argument("--beginning-low-ms", type=float, default=400.0)
    parser.add_argument("--beginning-pattern", action="store_true",
                        help="also accept the approximately 2 Hz startup pattern")
    parser.add_argument("--output", type=Path,
                        help="write every computed window to this CSV")
    parser.add_argument("--check", action="store_true",
                        help="run regression checks on annotated segments and events")
    arguments = parser.parse_args()
    if arguments.window_ms < 500.0:
        parser.error("--window-ms must be at least 500")
    if not 10.0 <= arguments.step_ms <= arguments.window_ms:
        parser.error("--step-ms must be between 10 and window-ms")
    if min(arguments.high_ms, arguments.steady_low_ms,
           arguments.beginning_low_ms) <= 0.0:
        parser.error("pattern durations must be positive")
    return arguments


def knee(value: float, low: float, high: float) -> float:
    return min(max((value - low) / (high - low), 0.0), 1.0)


def load_samples(path: Path) -> np.ndarray:
    """Return host time, unwrapped MCU time and scaled R/G/B/W samples."""
    samples: list[tuple[float, float, float, float, float, float]] = []
    group: dict[str, float] = {}
    previous_timestamp: int | None = None
    unwrapped_timestamp = 0

    with path.open(newline="") as capture_file:
        for row in csv.DictReader(capture_file):
            key = row["key"]
            value = float(row["value"])
            if key == "lrd":
                group = {"elapsed": float(row["elapsed_s"]), "lrd": value}
                continue
            if not group or key not in {"lgn", "lbl", "lwh", "ltu"}:
                continue
            group[key] = value
            if key != "ltu" or not all(
                    name in group for name in ("lrd", "lgn", "lbl", "lwh")):
                continue

            timestamp = int(group["ltu"])
            if previous_timestamp is not None:
                unwrapped_timestamp += (
                    timestamp - previous_timestamp
                ) % TIMESTAMP_MODULUS_US
            previous_timestamp = timestamp
            samples.append((
                group["elapsed"],
                unwrapped_timestamp * 1.0e-6,
                2.4 * group["lrd"],
                group["lgn"],
                1.3 * group["lbl"],
                group["lwh"],
            ))
            group = {}

    if len(samples) < 2:
        raise ValueError(f"{path}: no complete low-level RGBW groups")
    return np.asarray(samples, dtype=np.float64)


def pattern_detection(samples: np.ndarray, pattern: Pattern) -> Detection:
    """Score one window against one phase-independent asymmetric pattern."""
    mcu_time = samples[:, 1]
    rgb = samples[:, 2:5]
    rgb_ac = rgb - np.mean(rgb, axis=0)
    red_contrast = rgb_ac[:, 0] - 0.5 * (rgb_ac[:, 1] + rgb_ac[:, 2])
    contrast_energy = float(np.mean(red_contrast * red_contrast)) + 1.0

    best: Detection | None = None
    for offset_hz in (-0.2, -0.1, 0.0, 0.1, 0.2):
        frequency_hz = pattern.frequency_hz + offset_hz
        oscillator = np.exp(-2.0j * math.pi * frequency_hz * mcu_time)
        fundamental_rgb = np.mean(rgb_ac * oscillator[:, None], axis=0)
        harmonic_rgb = np.mean(rgb_ac * (oscillator * oscillator)[:, None],
                               axis=0)
        fundamental = fundamental_rgb[0] - 0.5 * (
            fundamental_rgb[1] + fundamental_rgb[2]
        )
        harmonic = harmonic_rgb[0] - 0.5 * (
            harmonic_rgb[1] + harmonic_rgb[2]
        )
        fundamental_amplitude = abs(fundamental)
        harmonic_amplitude = abs(harmonic)
        coherence = math.sqrt(min(
            2.0 * fundamental_amplitude * fundamental_amplitude /
            contrast_energy,
            1.0,
        ))
        harmonic_ratio = harmonic_amplitude / (fundamental_amplitude + 1.0)
        phase_alignment = float(
            (harmonic * np.conjugate(fundamental * fundamental)).real /
            (harmonic_amplitude * fundamental_amplitude *
             fundamental_amplitude + 1.0)
        )

        red_amplitude = abs(fundamental_rgb[0])
        inverse_red_amplitude = 1.0 / (red_amplitude + 1.0)
        green_projection = max(0.0, float(
            (fundamental_rgb[1] * np.conjugate(
                fundamental_rgb[0])).real * inverse_red_amplitude
        ))
        blue_projection = max(0.0, float(
            (fundamental_rgb[2] * np.conjugate(
                fundamental_rgb[0])).real * inverse_red_amplitude
        ))
        red_fraction = red_amplitude / (
            red_amplitude + green_projection + blue_projection + 1.0
        )

        ratio_error = (
            harmonic_ratio - pattern.expected_harmonic_ratio
        ) / 0.20
        ratio_score = 1.0 / (1.0 + ratio_error * ratio_error)
        phase_score = knee(phase_alignment, 0.40, 0.85)
        harmonic_shape = math.sqrt(ratio_score * phase_score)
        coherence_score = knee(coherence, 0.35, 0.65)
        red_score = knee(red_fraction, 0.55, 0.72)
        score = coherence_score * (0.5 + 0.5 * red_score) * (
            0.35 + 0.65 * harmonic_shape
        )
        candidate = Detection(
            elapsed_s=float(samples[-1, 0]),
            score=score,
            pattern=pattern.identifier,
            frequency_hz=frequency_hz,
            coherence=coherence,
            harmonic_ratio=harmonic_ratio,
            phase_alignment=phase_alignment,
            red_fraction=red_fraction,
            harmonic_shape=harmonic_shape,
        )
        if best is None or (candidate.score, candidate.coherence) > (
                best.score, best.coherence):
            best = candidate

    assert best is not None
    return best


def replay(samples: np.ndarray, window_s: float, step_s: float,
           patterns: list[Pattern]) -> list[Detection]:
    host_time = samples[:, 0]
    first_output = host_time[0] + window_s
    output_times = np.arange(first_output, host_time[-1], step_s)
    detections: list[Detection] = []
    for output_time in output_times:
        first = int(np.searchsorted(host_time, output_time - window_s,
                                    side="right"))
        last = int(np.searchsorted(host_time, output_time, side="right"))
        window = samples[first:last]
        if len(window) < 50:
            continue
        if window[-1, 0] - window[0, 0] < 0.90 * window_s:
            continue
        candidates = [pattern_detection(window, pattern)
                      for pattern in patterns]
        detections.append(max(
            candidates, key=lambda detection:
            (detection.score, detection.coherence)
        ))
    return detections


def write_detections(path: Path,
                     captures: Iterable[tuple[Path, list[Detection]]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as output_file:
        writer = csv.writer(output_file)
        writer.writerow([
            "capture", "elapsed_s", "score", "pattern", "frequency_hz",
            "coherence", "harmonic_ratio", "phase_alignment",
            "red_fraction", "harmonic_shape",
        ])
        for capture, detections in captures:
            for detection in detections:
                writer.writerow([
                    capture.name, f"{detection.elapsed_s:.6f}",
                    f"{detection.score:.6f}", detection.pattern,
                    f"{detection.frequency_hz:.6f}",
                    f"{detection.coherence:.6f}",
                    f"{detection.harmonic_ratio:.6f}",
                    f"{detection.phase_alignment:.6f}",
                    f"{detection.red_fraction:.6f}",
                    f"{detection.harmonic_shape:.6f}",
                ])


def percentile(values: list[float], quantile: float) -> float:
    return float(np.quantile(np.asarray(values), quantile))


def check_regressions(
        manifest: dict[str, dict[str, list[dict[str, float | str]]]],
        results: dict[str, list[Detection]], window_s: float) -> bool:
    values: dict[str, list[float]] = {
        "off": [], "beginning": [], "steady": [],
    }
    success = True
    print("\nAnnotated segments, steady pattern only:")
    for capture_name, annotations in manifest.items():
        detections = results.get(capture_name)
        if detections is None:
            continue
        for segment in annotations.get("segments", []):
            label = str(segment["label"])
            start = float(segment["start_s"]) + window_s
            end = float(segment["end_s"])
            segment_values = [
                detection.score for detection in detections
                if start <= detection.elapsed_s <= end
            ]
            if not segment_values:
                print(f"  {capture_name} {label}: no complete window")
                success = False
                continue
            values[label].extend(segment_values)
            print(
                f"  {capture_name} {label}: n={len(segment_values)} "
                f"min={min(segment_values):.3f} "
                f"p10={percentile(segment_values, 0.10):.3f} "
                f"median={percentile(segment_values, 0.50):.3f} "
                f"max={max(segment_values):.3f}"
            )

        for event in annotations.get("events", []):
            event_time = float(event["time_s"])
            maximum_delay = float(event["maximum_delay_s"])
            kind = str(event["kind"])
            threshold = 0.55 if kind == "steady_on" else 0.30
            matches = [
                detection for detection in detections
                if event_time <= detection.elapsed_s <=
                event_time + maximum_delay and
                ((kind == "steady_on" and detection.score >= threshold) or
                 (kind == "off" and detection.score < threshold))
            ]
            if matches:
                delay = matches[0].elapsed_s - event_time
                print(f"  {capture_name} {kind}: {delay:.3f} s")
            else:
                print(f"  {capture_name} {kind}: FAILED")
                success = False

    off_maximum = max(values["off"], default=1.0)
    beginning_maximum = max(values["beginning"], default=1.0)
    steady_minimum = min(values["steady"], default=0.0)
    print(
        "\nAggregate: "
        f"off max={off_maximum:.3f}, "
        f"beginning max={beginning_maximum:.3f}, "
        f"steady min={steady_minimum:.3f}, "
        f"steady p10={percentile(values['steady'], 0.10):.3f}"
    )
    success &= off_maximum < 0.20
    success &= beginning_maximum < 0.25
    success &= steady_minimum > 0.75
    return success


def check_beginning_mode(
        manifest: dict[str, dict[str, list[dict[str, float | str]]]],
        results: dict[str, list[Detection]], window_s: float) -> bool:
    values: dict[str, list[float]] = {
        "off": [], "beginning": [], "steady": [],
    }
    patterns: dict[str, list[int]] = {"beginning": [], "steady": []}
    for capture_name, annotations in manifest.items():
        detections = results.get(capture_name)
        if detections is None:
            continue
        for segment in annotations.get("segments", []):
            label = str(segment["label"])
            start = float(segment["start_s"]) + window_s
            end = float(segment["end_s"])
            selected = [
                detection for detection in detections
                if start <= detection.elapsed_s <= end
            ]
            values[label].extend(detection.score for detection in selected)
            if label in patterns:
                patterns[label].extend(
                    detection.pattern for detection in selected
                )

    off_maximum = max(values["off"], default=1.0)
    beginning_p10 = percentile(values["beginning"], 0.10)
    beginning_median = percentile(values["beginning"], 0.50)
    steady_minimum = min(values["steady"], default=0.0)
    beginning_selection = sum(
        pattern == BEGINNING_PATTERN.identifier
        for pattern in patterns["beginning"]
    ) / len(patterns["beginning"])
    steady_selection = sum(
        pattern == STEADY_PATTERN.identifier
        for pattern in patterns["steady"]
    ) / len(patterns["steady"])
    print(
        "\nStartup bank enabled: "
        f"off max={off_maximum:.3f}, "
        f"beginning p10={beginning_p10:.3f}, "
        f"beginning median={beginning_median:.3f}, "
        f"steady min={steady_minimum:.3f}, "
        f"pattern selection={beginning_selection:.3f}/"
        f"{steady_selection:.3f}"
    )
    return (
        off_maximum < 0.20 and
        beginning_p10 > 0.50 and
        beginning_median > 0.85 and
        steady_minimum > 0.75 and
        beginning_selection == 1.0 and
        steady_selection == 1.0
    )


def main() -> int:
    arguments = parse_arguments()
    with arguments.manifest.open() as manifest_file:
        manifest = json.load(manifest_file)

    captures = arguments.captures or [
        DEFAULT_CAPTURE_DIRECTORY / name for name in manifest
        if (DEFAULT_CAPTURE_DIRECTORY / name).exists()
    ]
    if not captures:
        print("no capture file found", file=sys.stderr)
        return 2

    window_s = arguments.window_ms * 0.001
    step_s = arguments.step_ms * 0.001
    steady_pattern = Pattern(
        STEADY_PATTERN.identifier, STEADY_PATTERN.name,
        arguments.high_ms, arguments.steady_low_ms)
    beginning_pattern = Pattern(
        BEGINNING_PATTERN.identifier, BEGINNING_PATTERN.name,
        arguments.high_ms, arguments.beginning_low_ms)
    selected_patterns = [steady_pattern]
    if arguments.beginning_pattern:
        selected_patterns.append(beginning_pattern)
    computed: list[tuple[Path, list[Detection]]] = []
    loaded_samples: dict[str, np.ndarray] = {}
    for capture in captures:
        samples = load_samples(capture)
        loaded_samples[capture.name] = samples
        detections = replay(samples, window_s, step_s, selected_patterns)
        computed.append((capture, detections))
        print(
            f"{capture.name}: {len(samples)} RGBW groups, "
            f"{len(detections)} detector windows"
        )

    if arguments.output:
        write_detections(arguments.output, computed)
        print(f"saved: {arguments.output.resolve()}")

    if arguments.check:
        if arguments.beginning_pattern:
            print("--check expects the competition default without startup pattern",
                  file=sys.stderr)
            return 2
        if (arguments.high_ms, arguments.steady_low_ms,
                arguments.beginning_low_ms) != (100.0, 233.0, 400.0):
            print("--check expects the nominal 100/233/400 ms timings",
                  file=sys.stderr)
            return 2
        result_map = {path.name: detections
                      for path, detections in computed}
        success = check_regressions(manifest, result_map, window_s)
        beginning_results = {
            name: replay(samples, window_s, step_s,
                         [steady_pattern, beginning_pattern])
            for name, samples in loaded_samples.items()
        }
        success &= check_beginning_mode(
            manifest, beginning_results, window_s)
        if not success:
            print("offline optical regression: FAILED", file=sys.stderr)
            return 1
        print("offline optical regression: PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
