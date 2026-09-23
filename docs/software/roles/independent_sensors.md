# Independent microphone, RGBW and distance roles

The acquisition code was extracted from `imav2026` into three roles. Each has
its own activation parameter, buffers, worker and recovery path. No beacon
detection, mission parameters, audio cadence, light pattern or IMAV dependency
is included. The sensor settings and publication periods below are read at
startup: save the parameters and reboot after changing them.

All three roles are compiled by default and **disabled at runtime**. Set
`ROLE.identification=false` to allow normal role startup.

| Sensor | Activation parameter | Hardware | Default publication |
| --- | --- | --- | --- |
| IM68A130A analog microphone | `ROLE.adc.microphone.im68a130` | PA4, ADC2, TIM6, DMA | 200 ms |
| OPT4060 RGBW | `ROLE.i2c.light.opt4060` | Shared I2C1, optional INT on PA8 | 100 ms |
| VL53L4CX range sensor | `ROLE.i2c.range.vl53l4cx` | Shared I2C1, address 0x29 | 200 ms |

Each enabled I2C sensor must answer its identity probe at startup. The analog
microphone has no identity probe: `mic.ok` reports ADC acquisition validity,
not physical microphone presence. As with existing roles, a
startup error is returned to `CANSlave::start()`, which stops starting further
roles and does not declare the node operational. Once started, runtime
acquisition failures are handled by the affected worker without stopping the
other roles; I2C bus recovery necessarily affects the shared bus.

## Microphone

The driver retains the PA4/ADC2_IN17 acquisition from IMAV: TIM6 trigger at
42.5 MHz / 1771 (approximately 24 kHz), oversampling x4 with a one-bit shift,
and a circular 1024-sample DMA buffer. It does not require I2C or PA8. ADC1
continues to monitor board voltage and temperature.

`role.adc.microphone.period_ms`: 50–1000 ms, default 200. The role publishes
`uavcan.protocol.debug.KeyValue` statistics from the latest complete
512-sample block (approximately 21.3 ms), rather than the entire publication
interval:

| Key | Meaning |
| --- | --- |
| `mic.ok` | 1 for a valid block, 0 after acquisition failure |
| `mic.dc` | Mean in 13-bit ADC counts |
| `mic.rms` | RMS amplitude after removing DC, in ADC counts |
| `mic.pp` | Peak-to-peak amplitude in ADC counts |
| `mic.clip` | Fraction of samples at or near the ADC rails, 0–1 |
| `mic.drop` | Cumulative discarded/missed blocks |
| `mic.rst` | Cumulative acquisition restarts |

These are electrical amplitude measurements, **not calibrated dB SPL**.
Errors or a 100 ms acquisition stall stop the timer before the ADC is reset.
The first block after each restart is discarded. A block is also discarded
if DMA advances while its statistics are being calculated. Invalid amplitude
fields are published as NaN instead of repeating a previous valid sample.

PA4 is configured as analog only when this role starts. It conflicts with
servo PWM CH6 and any external SPI chip select on that pin. The microphone
starts before optional roles to reserve its dynamically chosen ADC DMA stream;
DMA exhaustion is reported as `DMA_UNAVAILABLE`.

## OPT4060

The driver retains auto-range, 1.8 ms conversion per channel, RGBW burst reads,
CRC validation and per-channel rolling counters from IMAV. Corrupt frames and
frames with a channel that has not advanced are rejected.

| Parameter | Range/default |
| --- | --- |
| `role.i2c.light.opt4060.address` | 0x44–0x47, default 0x44; preferred address, then probe the others |
| `role.i2c.light.opt4060.period_ms` | 10–1000 ms, default 100; publication period |
| `role.i2c.light.opt4060.use_interrupt` | Default true: INT on PA8; false: poll every 10 ms without reserving PA8 |

With interrupts enabled, the worker waits for the end of all four channel
conversions; a 25 ms timeout permits a fallback read. Publication is throttled
independently of acquisition. Three consecutive read errors or 100 ms without
a fresh frame invalidate the measurements; initialization is retried every
five seconds while invalid telemetry continues.

`uavcan.protocol.debug.KeyValue` publications:

| Key | Meaning |
| --- | --- |
| `opt.ok` | 1 for fresh, non-overloaded RGBW data; otherwise 0 |
| `opt.red`, `opt.green`, `opt.blue`, `opt.clear` | Linear ADC codes (`mantissa << exponent`), transmitted as float32; NaN when invalid |
| `opt.ovf` | Overload flag from the most recently decoded frame |

The codes are uncalibrated sensor readings, not lux. Float32 transmission may
round the least significant bits of large expanded ADC codes. Data-ready on
PA8 conflicts with servo PWM CH1 or a DShot configuration reserving PA8;
polling mode leaves PA8 available. No board-level pin mapping is changed.

## VL53L4CX

The ST component is included under `third_party/vl53l4cx`, with its license and
[provenance](../../../third_party/vl53l4cx/README.microcan.md). It uses long-range
mode, a 30 ms timing budget and asynchronous one-shot measurements. The worker
selects the nearest target with a valid ST status.

| Parameter | Range/default |
| --- | --- |
| `role.i2c.range.vl53l4cx.period_ms` | 100–1000 ms, default 200 |
| `role.i2c.range.vl53l4cx.sensor_id` | 0–255, default 0 |

Output: `uavcan.equipment.range_sensor.Measurement`, with distance in metres,
sensor type LIDAR, field of view inherited from IMAV (0.314159 rad), undefined
body orientation and timestamp 0 (no synchronized network time). A failed
read or a result with no valid target produces `READING_TYPE_UNDEFINED` and
NaN. It never presents an old measurement or a fabricated maximum distance as
a valid range. Three consecutive acquisition errors trigger reinitialization
attempts every five seconds; invalid measurements continue at the configured
publication period.

The role needs neither the microphone nor OPT4060. It does not pause the RGBW
sensor during infrared emission: simultaneous operation measures the actual
light present, including any optical coupling from the ToF module. Applications
requiring optical isolation need coordinated acquisition above these drivers.

## Shared I2C and resource ownership

Both I2C roles call `I2CPeriph::start()` explicitly. The first call reserves
I2C1, PA15/SCL and PB07/SDA, configures pull-ups and starts the driver. Further
calls succeed without starting it again. This is idempotent initialization,
not automatic dependency resolution. Role startup is sequential; this helper
is not a concurrent or reference-counted peripheral lifecycle manager.

Use `bus.i2c.frequency_khz=100` or `400` for these sensors. Both roles reject
the 1 MHz setting. `bus.i2c.pullup_resistor` controls the board pull-ups. The
I2C owner, rather than each individual sensor, reserves the shared pins. They
therefore coexist with the MPL3115A2 and QMC5883, subject to bus loading, but
conflict with PWM CH7, the external WS2812 strip and the voltmeter LEDs on PB07.

Every transaction owns the I2C mutex, including recovery after timeout or bus
error. `reset()` takes that mutex; `resetLocked()` must be used only when it is
already held. A normal address NACK during probing does not reset the bus.
The imported recovery restores only SDA/SCL GPIO state, uses open-drain SCL
and issues a STOP after SDA is released.

## Build and validation

### RAM allocation

`addRole()` creates the role object only if its activation parameter is true.
Acquisition buffers are allocated in `start()`, and worker stacks are created
with `chThdCreateFromHeap()`. There are no permanent sample/result buffers
for these roles. The ST result buffer and mutable tuning table belong to its
dynamically allocated device context; constant configurations/defaults reside
in flash.

This does not mean zero static RAM when a role is compiled but disabled at
runtime: the existing CRTP callback pointers use 4 bytes per role; ChibiOS
reserves its ADC2 and TIM6 driver structures (56 and 20 bytes in the debug
build), and the shared parameter store and DroneCAN transfer IDs also retain
their entries. Compile-time switches remove the corresponding role code and
entries; disabling the microphone also removes the ADC2/TIM6 structures.

### Checks

The switches are `USE_MICROPHONE_ROLE`, `USE_OPT4060_ROLE` and
`USE_VL53L4CX_ROLE` in `COMMON/source/roleConf.h`. They can also be overridden
on the application make command line, for example:

```sh
make -C microcan clean
make -C microcan USE_MICROPHONE_ROLE=0 USE_OPT4060_ROLE=1 USE_VL53L4CX_ROLE=0
python3 tests/sensor_roles/run.py
```

Use a clean build or a separate `BUILDDIR`/`DEPDIR` when changing make
overrides, because ChibiOS does not track command-line flag changes.

Disabling the microphone also excludes ADC2/TIM6 HAL support. Disabling the
VL53L4CX excludes the ST sources and headers from the build. The host tests
exercise audio statistics, RGBW CRC/freshness, counter wrap, and the ST port's
endianness, buffer limits and error propagation.

Hardware acceptance still requires each role alone, all three together, I2C
bus recovery alongside an existing sensor, PA4/PA8/PB07 conflict checks,
sensor disconnect/reconnect, and the ADC1 battery/temperature alarm path.
