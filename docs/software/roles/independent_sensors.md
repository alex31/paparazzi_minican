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
microphone has no identity probe: the spectrum `STATUS_VALID` flag reports ADC acquisition validity,
not physical microphone presence. As with existing roles, a
startup error is returned to `CANSlave::start()`, which stops starting further
roles and does not declare the node operational. Once started, runtime
acquisition failures are handled by the affected worker without stopping the
other roles; I2C bus recovery necessarily affects the shared bus.

## Microphone

**Hardware status: untested.** The FFT and compact Spectrum publication have
passed software tests and firmware compilation, but have not been validated
on the board with the microphone and a physical CAN/CAN FD bus.

The driver retains the PA4/ADC2_IN17 acquisition from IMAV: TIM6 trigger at
42.5 MHz / 1771 (approximately 24 kHz), oversampling x4 with a one-bit shift,
and a circular 1024-sample DMA buffer. It does not require I2C or PA8. ADC1
continues to monitor board voltage and temperature.

`role.adc.microphone.period_ms`: 50–1000 ms, default 200. The role publishes
[`microcan.audio.Spectrum`](../../../DSDL/microcan/audio/20900.Spectrum.uavcan)
(message ID **20900**), replacing the earlier `mic.*` KeyValue messages.
The activation parameter keeps its existing name for compatibility; the
spectrum format does not depend on the microphone model or an IMAV mission.

Processing uses the latest complete **512-sample block**, approximately
**21.3 ms**, at each publication deadline. It removes the block's mean,
applies a periodic Hann window and computes an in-place radix-2 FFT.
It does not average the entire publication interval and does not compute
FFTs for blocks that will not be published. Publication happens on a DMA
block boundary, so the configured period is a minimum interval.

A real 512-sample input has 257 nonredundant bins. Their spacing is
**46.8706 Hz** at the actual sample rate of 23997.741 Hz. Selection retains
only centres in [100, 3000] Hz: bins 3 through 64, **62 candidates**, with
centres approximately 140.612 through 2999.718 Hz. Up to **ten strongest bins**
are sent in descending amplitude order (ties by ascending frequency).
These are FFT bins, not ten distinct tones: the Hann window spreads a tone
across neighbouring bins. There is no interpolation or peak merging.

| Message field | Meaning |
| --- | --- |
| `adc_bits` | Nominal ADC output width, currently 13; sets the logarithmic reference |
| `status` | Bit 0: valid; bit 1: near-rail samples; bit 2: acquisition loss/restart since previous publication attempt |
| `bins[]` | Up to ten pairs: `uint16 frequency_hz`, `uint16 level` |

Frequencies are rounded to the nearest hertz. The level uses all 16 unsigned
bits for a logarithmic amplitude span from one ADC count peak (code 0) to
full-scale peak (code 65535); both endpoints saturate. With the current
13-bit ADC output, this spans approximately **−72.2472 to 0 dBFS**:
`dBFS = 72.2472*(level/65535 - 1)`. A half-scale sinusoid reads about
code 60074, corresponding to −6.02 dBFS. `adc_bits` allows other producers
to use a different nominal output width without changing the format.

Before quantization, amplitude is corrected for Hann coherent gain:
`20*log10(4*abs(FFT[k])/(512*4096))`. These are individual bin amplitudes,
not power spectral density, calibrated dB SPL, or a measured ADC noise/SNR
specification. Tones between bins have lower individual bin levels, and no
microphone frequency-response correction is applied. Bins at/below one ADC
count peak are omitted. Silence produces an empty vector with `STATUS_VALID`
set; noise may fill all ten entries. FFT size, sample rate, sequence and
cumulative counters are no longer repeated in the compact message.

Errors or a 100 ms acquisition stall stop the timer before the ADC is reset.
The first block after each restart is discarded. Before the FFT, samples are
copied into private working memory and rejected if DMA advances during the
copy. Invalid acquisition publishes an empty vector without `STATUS_VALID`,
rather than repeating a previous spectrum. Intentional skips between
publication deadlines do not count as acquisition loss. Internal loss/restart
counters set `STATUS_DISCONTINUITY` on the next publication attempt.

The dedicated DSDL definitions and generated C bindings are versioned in
[DSDL](../../../DSDL/README.md). Receivers must load the `microcan` namespace.
The generic format accepts **255 bins**, derived from libcanard's 1023-byte
transfer limit: `(1023 - 2 header bytes - 1 count byte) / 4 bytes per bin`.
This role still selects at most ten, for **42 bytes with TAO / 43 without**.
libcanard handles physical CAN/CAN FD fragmentation. No raw audio is transmitted.
The bounded send-side view reserves only ten entries and a 43-byte encoded
buffer, so the maximum wire capacity does not increase the worker stack.

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
in flash. The microphone adds **4104 bytes** of FFT state (two 512-float
arrays plus two scalars) on the regular heap only at activation. Its DMA
state remains 2060 bytes, and its worker stack remains 1536 bytes plus RTOS
thread metadata. No allocation takes place per FFT or per publication.

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
compare the FFT against an independent double-precision DFT, verify dBFS
scaling, DC rejection, band limits, top-ten ranking and clipping, and check
Spectrum encoding/decoding with and without tail-array optimisation,
uint16 scaling, bounded/canonical encoder equivalence, and libcanard
fragmentation/reassembly through the full 255-bin capacity on CAN and CAN FD. They
also cover RGBW CRC/freshness, counter wrap and the ST port's endianness,
buffer limits and error propagation.

Hardware acceptance still requires each role alone, all three together, I2C
bus recovery alongside an existing sensor, PA4/PA8/PB07 conflict checks,
sensor disconnect/reconnect, and the ADC1 battery/temperature alarm path.
For audio, inject known tones, check the dBFS reference against ADC amplitude,
and measure execution time and dropped-block counters under concurrent load.
