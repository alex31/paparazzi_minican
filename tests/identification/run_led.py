#!/usr/bin/env python3
"""Run the production RGB animation against a clock/LED double (no hardware)."""
from pathlib import Path
import subprocess
import tempfile

project = Path(__file__).resolve().parents[2]
tests = Path(__file__).parent
# Use the production color conversion without pulling in hardware-dependent
# parts of the shared LED transport header.
shared = project.parents[2] / 'COMMON/various/led2812.hpp'
colors = shared.read_text()
start = colors.index('constexpr RGB hsv2rgb(HSV in)')
colors = colors[start:colors.index('\n}', start) + 2]
with tempfile.TemporaryDirectory(prefix='identification-led-') as directory:
    binary = Path(directory) / 'led_test'
    (Path(directory) / 'color_helpers.hpp').write_text(colors)
    subprocess.run(['c++', '-std=c++20', '-g', '-O0', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=undefined', '-fno-sanitize-recover=undefined',
                    '-I' + directory,
                    '-I' + str(tests / 'led_stubs'),
                    '-I' + str(project / 'COMMON/source'),
                    str(tests / 'led_test.cpp'),
                    str(project / 'COMMON/source/rgbLeds.cpp'),
                    '-o', str(binary)], check=True)
    for scenario in ('digits', 'wrap', 'transport', 'initially-on', 'identification', 'wheel',
                     'motif', 'minimal', 'constant', 'constant-off', 'constant-minimal'):
        subprocess.run([str(binary), scenario], check=True)
    # Activation during each digit, each separator and the long end-of-ID pause.
    for ms in (1, 150, 199, 501, 600, 1000, 1400, 1800, 2101, 2400, 3390):
        subprocess.run([str(binary), 'toggle', str(ms)], check=True)
