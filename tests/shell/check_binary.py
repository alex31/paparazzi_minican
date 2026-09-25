#!/usr/bin/env python3
"""Reject the legacy static shell buffers and print stack in a linked ARM ELF."""
import argparse
import re
import subprocess
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--noshell', action='store_true')
parser.add_argument('elf', type=Path)
args = parser.parse_args()
symbols = subprocess.check_output(['arm-none-eabi-nm', '-S', '-C', str(args.elf)], text=True)
for forbidden in ('waSerialPrint', 'complWorlds', 'cpu_window', 'threadCpuInfo',
                  'shell_terminated', 'legacy_chprintf', 'legacy_chvprintf'):
    assert forbidden not in symbols, f'Unexpected legacy allocation/code: {forbidden}'
assert not re.search(r' [bBdD] rl$', symbols, re.M), 'Static microrl context survived'
if args.noshell:
    assert 'ShellRole::' not in symbols and 'shellContext' not in symbols
    assert 'shellWorker' not in symbols and 'printWorker' not in symbols
    strings = subprocess.check_output(['arm-none-eabi-strings', str(args.elf)], text=True)
    assert 'ROLE.shell' not in strings.splitlines(), 'Shell parameter in NOSHELL build'
else:
    assert 'ShellRole::start' in symbols and 'shellContext' in symbols
    assert 'consolePrintfStart' in symbols and 'printWorker' in symbols
print(f'{args.elf}: shell allocation layout OK')
