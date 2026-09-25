#!/usr/bin/env python3
"""Test real shell startup rollback, command callbacks and printf bounds on host."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
console = (root / 'microcan/source/ttyConsole.cpp').read_text()
start = console.index('DeviceStatus ShellRole::subscribe')
end = console.index('\nnamespace {', start)
lifecycle = console[start:end]
start = console.index('  void shellPrint(')
end = console.index('  THD_FUNCTION(shellWorker', start)
callbacks = console[start:end]
printer = (root / 'microcan/source/consolePrintf.cpp').read_text()
printer = '\n'.join(line for line in printer.splitlines() if not line.startswith('#include'))
harness = Path(__file__).with_name('shell_test.cpp').read_text()
harness = harness.replace('@PRINTER@', printer).replace('@CALLBACKS@', callbacks)
harness = harness.replace('@LIFECYCLE@', lifecycle)
with tempfile.TemporaryDirectory(prefix='shell-test-') as directory:
    source = Path(directory) / 'shell_test.cpp'
    source.write_text(harness)
    binary = Path(directory) / 'shell_test'
    subprocess.run(['c++', '-std=c++23', '-g', '-O0', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=undefined', '-fno-sanitize-recover=undefined',
                    str(source), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
