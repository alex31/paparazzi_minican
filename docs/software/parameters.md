# Parameter System

Parameter list: [COMMON/source/nodeParameters.hpp](../../COMMON/source/nodeParameters.hpp).
Stored in external EEPROM via Persistant::Storage and exposed via UAVCAN GetSet.

Important parameters:

- uavcan.node_id (0 = dynamic ID)
- uavcan.dynid.fd (enable FD dynamic allocation)
- uavcan.param_set_behavior (SetRam / SetRamFlash / SetRamFlashAndReboot)
- bus.serial.baudrate (0..460800, where 0 enables GNSS UBX auto-baud probing)
- bus.i2c.frequency_khz
- ROLE.imav.beacon (enable the IMAV sound/light role)
- role.imav.audio.band_low_hz (2000..2600 Hz, default 2000; applied when the
  IMAV role starts, so reboot after changing it)
- role.imav.light.i2c_address (0x44..0x47, default 0x44 for OPT4060 ADDR=GND)
- role.imav.time_of_flight (enable the optional VL53L4CX, default false)
- role.imav.tof.period_ms (100..1000 ms, used only when time-of-flight is
  enabled; default 200 ms before deterministic jitter)
- role.imav.debug.publish.optional (publish the optional IMAV tuning keys in
  addition to the mandatory `det`/`snr`/`lit` values; derived tuning remains
  at 5 Hz while lossless RGBW samples are sent at 100 Hz; default false,
  applied live)
- ROLE.* toggles for each role

Parameter handling is implemented in:
- [microcan/source/UAVCanSlave.cpp](../../microcan/source/UAVCanSlave.cpp) (GetSet, ExecuteOpcode)
- [COMMON/source/deviceResource.cpp](../../COMMON/source/deviceResource.cpp) (storage instance)
