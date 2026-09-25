# Shell role regression tests

Run `python3 tests/shell/run.py`. It compiles the actual production startup,
command callbacks and printf worker against host hardware/RTOS doubles, with
undefined-behavior sanitization. It covers all three fallible allocations,
UART failure, resource conflict, successful startup, disabled output, bounded
formatting, case-insensitive dispatch and completion termination.

The fake RTOS dispatches one print message at a time. This exercises buffer
bounds and ownership; it does not validate preemption, physical UART behavior
or stack depth. Test T17 in `procedure_de_test.md` covers the board checks.
The identification suite separately checks activation with/without TRACE,
shell off/on in both modes, and suppression of other roles while identifying.

After firmware builds, run:

```sh
python3 tests/shell/check_binary.py microcan/build_MICROCAN/MICROCAN.elf
python3 tests/shell/check_binary.py --noshell /tmp/microcan-shell-noshell/MICROCAN.elf
```

The ELF checks ensure that the old shell/printf static buffers are actually
removed by the linker and that NOSHELL excludes the role and its parameter.
