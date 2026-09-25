Run from the project root:

```sh
python3 tests/dshot_role/run.py
```

The host C++ test exercises the actual `EscDshot::periodic` function with
hardware and RTOS calls stubbed and UBSan enabled. It checks per-cycle response
processing, publication dividers 0/1/3, EDT between publication cycles,
non-contiguous channel mapping, stopped motors and intervals without valid RPM.
It builds with EDT on, EDT off and bidirectional DShot off. `--cxx` selects the
host C++ compiler. This does not exercise the shared decoder, DMA or RTOS timing.
