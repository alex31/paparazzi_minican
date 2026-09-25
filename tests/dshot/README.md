Run from the project root:

```sh
python3 tests/dshot/run.py
```

The test uses the shared sources in `~/DEV/STM32/CHIBIOS/COMMON/various`
(`--various PATH` overrides this location) and a host C compiler with UBSan.
It compiles the actual capture-decoding function and eRPM codec with RTOS
headers stubbed, without adding a firmware test interface.

Coverage: all 4096 telemetry payloads, CRC rejection, poisoned unwritten
buffer entries, timestamp jitter and 16-bit wraparound, stopped-motor and
zero-period values, and malformed captures. This does not exercise hardware
DMA, timer timing or RTOS scheduling.

The shared driver keeps TX DMA allocated between frames and no longer clears
the capture buffer before each frame. Its synchronous API and timer sequence
are retained. Measure CPU time and telemetry errors on the board to quantify
the effect; host tests do not establish a performance gain.
