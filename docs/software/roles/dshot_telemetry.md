# DShot telemetry: freshness and FlexDebug

The shared driver in `~/DEV/STM32/CHIBIOS/COMMON/various/esc_dshot.{h,c}`
tracks receipt and age separately for temperature, voltage, current, consumption,
serial RPM, EDT stress and EDT status. These metadata are internal; the legacy
10-byte serial frame is unchanged.

`dshotGetTelemetry()` returns a snapshot with `valid_mask` and `updated_at[]`.
Use `dshotTelemetryIsValid()` on that snapshot before using a measurement. A field
is valid after reception until its age reaches `DSHOT_TELEMETRY_TIMEOUT_MS`
(3000 ms by default). Zero is a valid measurement. Receiving a different field,
an ignored EDT debug frame or a failed serial query does not refresh its age.
`updated_mask` identifies fields in the last decoded EDT/serial update; it is
not an indication that every subsequent read contains new data.

MicroCAN publishes the standard `uavcan.equipment.esc.Status`. Its floating-point
fields use NaN for unavailable/expired data, including when EDT is compiled out.
Valid temperature is in kelvins, voltage in volts and current in amperes.
Timer eRPM remains separate from the serial RPM cache: Status is sent only if
a valid eRPM arrived during the current `rpm_freq_div` interval. Its RPM is
`eRPM / (motor_poles / 2)`; zero RPM is valid. No RPM is fabricated when only EDT
arrives. Setting `rpm_freq_div` to zero disables both CAN publications while
continuing to decode incoming responses.

## MicroCAN FlexDebug encoding, version 1

This uses the existing `dronecan.protocol.FlexDebug` type, message type ID 16371.
The payload below is a **private MicroCAN convention**, not an upstream DShot
or DroneCAN standard. Inner IDs `2000 + esc_index` are a local choice, not a
registered reservation. Coordinate their allocation before adopting this
convention upstream; a receiver must identify the MicroCAN node as well as the ID.
No DSDL definition or generated codec has been added or changed.

Shared definitions/codecs used by the build:

- `~/DEV/STM32/UAVCAN/DSDL/dronecan/protocol/16371.FlexDebug.uavcan`
- `~/DEV/STM32/UAVCAN/DSDL/uavcan/equipment/esc/1034.Status.uavcan`
- `~/DEV/STM32/UAVCAN/DSDLC/include/dronecan.protocol.FlexDebug.h`

`id = 2000 + esc_index`, where the zero-based index is the same as in Status
(`role.esc.dshot.map_index1 + active-channel slot`). Physical channel gaps do not
change this mapping. Separate IDs let receivers cache each ESC independently.
`u8.len = 5`:

| Byte | Meaning |
| --- | --- |
| 0 | Encoding version: `1` |
| 1 | ESC index, matching `id - 2000` |
| 2 | Presence: bit 0 = stress included; bit 1 = status included; other bits zero |
| 3 | Maximum raw EDT stress (0–255) observed in the reporting window |
| 4 | EDT status: bits 7/6/5 = alert/warning/error observed in the window; bit 4 zero; bits 3–0 = maximum of the received four-bit stress values (0–15) |

Stress scales are kept as received; they are not percentages. An absent field's
byte is zero and must be ignored. A present zero value is valid. For example,
`id=2004`, `u8=[01 04 03 64 EB]` reports ESC 4, raw stress maximum 100,
all three events, and four-bit stress maximum 11.

A reporting window ends when the transfer is accepted by the local CAN queue.
Maxima and event bits accumulate until then; a queue rejection retains them for
retry. Inclusion requires at least one new report in that window and a latest
report less than 3 seconds old for that field. Expired fields are discarded;
cached values are not sent repeatedly without new reports. Presence describes
this window, so consumers should update their own per-field receipt timestamps
and expire old diagnostics rather than interpret an absent field as zero.

Transfers use low priority, at a Status publication opportunity, with at least
100 ms between opportunities for FlexDebug. They can be sent even without valid
RPM. At most four transfers per 100 ms are added for four motors. With classic
CAN and tail-array optimization, the two-byte ID and five-byte payload fit in
one CAN frame including its transfer tail byte. Queue acceptance does not
guarantee delivery or consumption by a receiver.

ArduPilot's FlexDebug receiver stores the latest value by source node and inner
ID. Access requires scripting support and `CAN_Dx_OPTIONS` bit 9
(`ENABLE_FLEX_DEBUG`). A decoder/Lua script for **this** format is still needed;
the existing AM32 FlexDebug example uses a different format. The generic receiver
does not automatically feed these bytes into ArduPilot's native ESC stress logs.

## Verification

`python3 tests/dshot_role/run.py` exercises the actual role loop and shared
metadata/serial functions on the host with RTOS calls stubbed, under UBSan.
It covers independent expiry, valid zeros, timer wrap, serial failures, channel
mapping, NaN, and FlexDebug aggregation, rate limiting and queue retries, plus
build variants with and without bidirectional DShot/EDT.
The same command checks NaN preservation through the shared float16 codec and
FlexDebug encoding/decoding with and without tail-array optimization.
