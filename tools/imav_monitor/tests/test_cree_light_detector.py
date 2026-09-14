"""Optical behavior of the bench profile and competition isolation.

Run: python3 -m unittest discover -s tools/imav_monitor/tests -p 'test_*.py'
"""

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from offline_light_detector import (  # noqa: E402
    BEGINNING_PATTERN, CREE_PATTERN, STEADY_PATTERN,
    DETECTION_ENTER_SCORE, DETECTION_EXIT_SCORE,
    replay, synchronized_events,
)


def lamp_samples(frequency=7.9904, duty=0.5, color=(1.0, 1.0, 1.0)):
    # OPT4060 groups take about 7.2 ms; RGB channels convert sequentially.
    time = np.arange(0.0, 4.0, 0.0072)
    rgb_time = time[:, None] + np.array([0.0, 0.0018, 0.0036])
    on = ((rgb_time - 0.039) * frequency) % 1.0 < duty
    noise = np.random.default_rng(20260911).normal(0.0, 20.0, on.shape)
    rgb = np.array([18000.0, 22000.0, 16000.0]) + noise
    rgb += 30000.0 * on * np.asarray(color)
    return np.column_stack((time, time, rgb, np.mean(rgb, axis=1)))


class CreeLightDetectorTest(unittest.TestCase):
    def test_white_and_red_filtered_headlamp_detected(self):
        for color in ((1, 1, 1), (1, 0.08, 0.03)):
            for duty in (0.48, 0.50, 0.54):
                with self.subTest(color=color, duty=duty):
                    detections = replay(lamp_samples(duty=duty, color=color),
                                        1.0, 0.05, [CREE_PATTERN])
                    self.assertTrue(detections)
                    self.assertGreater(min(d.score for d in detections),
                                       DETECTION_ENTER_SCORE)

    def test_competition_rejects_headlamp(self):
        for color in ((1, 1, 1), (1, 0.08, 0.03)):
            for pattern in (STEADY_PATTERN, BEGINNING_PATTERN):
                with self.subTest(color=color, pattern=pattern.name):
                    detections = replay(lamp_samples(color=color),
                                        1.0, 0.05, [pattern])
                    self.assertLess(max(d.score for d in detections),
                                    DETECTION_EXIT_SCORE)

    def test_wrong_cadences_rejected_in_cree_mode(self):
        for frequency in (2.0, 3.0, 4.0, 6.0, 10.0, 12.0):
            with self.subTest(frequency=frequency):
                detections = replay(lamp_samples(frequency=frequency),
                                    1.0, 0.05, [CREE_PATTERN])
                self.assertLess(max(d.score for d in detections),
                                DETECTION_ENTER_SCORE)

    def test_constant_light_and_sensor_noise_rejected(self):
        samples = lamp_samples(duty=1.0)
        for pattern in (CREE_PATTERN, STEADY_PATTERN):
            with self.subTest(pattern=pattern.name):
                detections = replay(samples, 1.0, 0.05, [pattern])
                self.assertLess(max(d.score for d in detections),
                                DETECTION_EXIT_SCORE)

    def test_tiny_periodic_noise_on_bright_background_rejected(self):
        samples = lamp_samples(duty=1.0)
        samples[:, 2:5] += 20.0 * np.sin(2.0 * np.pi * 8.0 * samples[:, 0, None])
        detections = replay(samples, 1.0, 0.05, [CREE_PATTERN])
        self.assertLess(max(d.score for d in detections), DETECTION_EXIT_SCORE)

    def test_events_follow_headlamp_falling_edges(self):
        samples = lamp_samples()
        detections = replay(samples, 1.0, 0.05, [CREE_PATTERN])
        events = synchronized_events(samples, detections, [CREE_PATTERN])
        active_times = np.array([e.elapsed_s for e in events if e.active])
        self.assertGreater(len(active_times), 20)
        # Initial lock may arrive partway through the 20 ms event window.
        self.assertAlmostEqual(active_times[1] - active_times[0],
                               1.0 / 7.9904, delta=0.020)
        np.testing.assert_allclose(np.diff(active_times[1:]), 1.0 / 7.9904,
                                   atol=0.01)
        since_rise = (active_times - 0.039) % (1.0 / 7.9904)
        edge_errors = since_rise - 0.5 / 7.9904
        self.assertLess(float(np.max(np.abs(edge_errors))), 0.020)

    def test_signal_disappears_after_one_window(self):
        samples = lamp_samples()
        samples[samples[:, 0] >= 2.0, 2:] = 20000.0
        detections = replay(samples, 1.0, 0.05, [CREE_PATTERN])
        self.assertLess(max(d.score for d in detections if d.elapsed_s >= 3.0),
                        DETECTION_EXIT_SCORE)


if __name__ == '__main__':
    unittest.main()
