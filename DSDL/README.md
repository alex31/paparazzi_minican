# MicroCAN message definitions

`microcan.audio.Spectrum` (vendor-specific message ID **20900**) carries a
variable vector of frequency/amplitude pairs. Its format does not prescribe
a microphone model, FFT length, sampling rate, frequency band or bin count.
The [schema](microcan/audio/20900.Spectrum.uavcan) and
[bin type](microcan/audio/Bin.uavcan) define the wire representation.
This ID is unused by the shared DSDL tree at integration time; deployments
with additional vendor messages must keep their ID assignments consistent.

## Payload and capacity

Each bin contains **two uint16 fields**, hence **4 bytes**:

- `frequency_hz`: centre frequency rounded to the nearest hertz (0–65535).
- `level`: normalized logarithmic amplitude (0–65535), defined below.

The header contains `uint8 adc_bits` and `uint8 status`. A variable-length
array of up to 255 bins needs an 8-bit count when tail-array optimisation
(TAO) is disabled. The payload limit comes from the project's libcanard
`CANARD_MAX_TRANSFER_PAYLOAD_LEN`, currently **1023 bytes**:

```text
max_bins = floor((max_transfer_payload - header_bytes - count_bytes) / bin_bytes)
         = floor((1023 - 2 - 1) / 4)
         = 255
payload without TAO = 3 + 4*N bytes
payload with TAO    = 2 + 4*N bytes
```

At 255 bins this is **1023 bytes**, or **1022 with TAO**. This limit concerns
an entire DroneCAN transfer, not a physical CAN frame. libcanard performs
CAN/CAN FD fragmentation and reassembly. The capacity is the same for both
buses. [DroneCAN TAO](https://dronecan.github.io/Specification/3._Data_structure_description_language/#tail-array-optimization)
omits the final array count where possible;
[CAN FD encoding](https://dronecan.github.io/Specification/4.4_CANFD_bus_transport_layer/)
disables TAO.

[`SpectrumWire`](../COMMON/source/spectrumMessage.hpp) derives the capacity
from the library limit and checks the generated DSDL maximum at compile time.
The microphone currently selects **ten strongest bins**, an application choice
independent of the 255-bin wire capacity. Its payload is therefore **42 bytes
with TAO / 43 without**, versus the previous 81/82-byte floating-point format.
The FFT remains 512 points, over 100 Hz–3 kHz.

## Unsigned logarithmic amplitude

`adc_bits` describes the nominal output word width (2–32 bits), including the
DC midpoint/sign bit. The full-scale peak amplitude of the centred signal is
`2^(adc_bits-1)` ADC counts. The encoding spans **one ADC count peak to
full-scale peak**; both endpoints saturate:

```text
range_db = 20*log10(2^(adc_bits-1)) = 6.020599913*(adc_bits-1)
level = round(65535 * clamp(1 + amplitude_dbfs/range_db, 0, 1))
amplitude_dbfs = range_db * (level/65535 - 1)
```

For the current 13-bit oversampled output, `range_db = 72.2472 dB`:

| Input amplitude | Level code | Interpretation |
| --- | --- | --- |
| Zero or ≤1 ADC count peak | 0 | At/below −72.2472 dBFS |
| 1024 ADC counts peak | ≈54613 | −12.0412 dBFS |
| 2048 ADC counts peak | ≈60074 | −6.0206 dBFS |
| ≥4096 ADC counts peak | 65535 | At/above 0 dBFS |

One code step represents about 0.00110 dB for this reference range; that is
encoding granularity, not measurement accuracy. This nominal amplitude span
is **not measured ADC noise, ENOB, SNR or calibrated sound pressure**. The
microphone omits bins at/below one ADC count peak. Other producers may retain
zero codes, which indicate underflow/floor, not invalid acquisition.

`status` is a bit mask: `STATUS_VALID` (1), `STATUS_CLIPPED` (2), and
`STATUS_DISCONTINUITY` (4, acquisition loss/restart since the previous
publication attempt). An invalid acquisition has an empty vector. An empty
vector with VALID set means no bins above the producer's floor. Spectrum
metadata and cumulative diagnostics are not repeated in this compact format;
the transport already provides node identity and a transfer ID.

## Receivers

Load `DSDL/microcan` alongside the standard DroneCAN definitions. From the
repository root, for example:

```python
import dronecan

dronecan.load_dsdl("DSDL/microcan")
Spectrum = dronecan.thirdparty.microcan.audio.Spectrum

def on_spectrum(event):
    msg = event.message
    if not (msg.status & msg.STATUS_VALID) or not 2 <= msg.adc_bits <= 32:
        return
    range_db = 6.020599913 * (msg.adc_bits - 1)
    for frequency_bin in msg.bins:
        dbfs = range_db * (frequency_bin.level / 65535.0 - 1.0)
        print(frequency_bin.frequency_hz, dbfs)
        # Codes 0 and 65535 are saturated bounds, not exact measurements.

# node.add_handler(Spectrum, on_spectrum)
```

Reload/regenerate the definitions after this format change: its type signature
has changed from the initial floating-point draft.

## Bindings and memory

`generated/include` and `generated/src` are supplied with the schema, so normal
firmware builds do not need a Python code generator. Regenerate them with the
shared `UAVCAN/dronecan_dsdlc` compiler after any schema change:

```sh
python3 tools/generate_audio_dsdl.py
python3 tools/generate_audio_dsdl.py --check
```

That compiler needs its Python dependencies, including Empy 3.x.
`--compiler /path/to/dronecan_dsdlc.py` overrides the shared checkout path.
`make -C microcan build_audio_dsdl` is an equivalent generation target.
Only the two audio bindings are copied; the shared `dronecan_msgs.h` remains
the aggregate header for standard messages.

The canonical generated C structure/codec supports all 255 bins. The firmware
uses [`SpectrumMessage<10>`](../COMMON/source/spectrumMessage.hpp), a bounded
send-side view of exactly the same schema: a 44-byte object and a maximum
43-byte encoded buffer. This avoids reserving a 1024-byte message and a
1023-byte encoding buffer for a ten-bin publication. No persistent static
buffers or extra per-publication heap allocations are introduced. Use a
single producer specialization per node/subject, because the node's existing
publish template keeps a separate transfer-ID counter for each C++ type.

`python3 tests/sensor_roles/run.py` checks the bounded encoder byte-for-byte
against the canonical one, logarithmic scale endpoints and quantization,
all lengths through 255, and actual libcanard fragmentation/reassembly in
CAN and CAN FD (including maximum payload and FD padding).
