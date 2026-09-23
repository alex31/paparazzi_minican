# Parameter System

Parameter list: [COMMON/source/nodeParameters.hpp](../../COMMON/source/nodeParameters.hpp).
Stored in external EEPROM via Persistant::Storage and exposed via UAVCAN GetSet.

Important parameters:
- uavcan.node_id (0 = dynamic ID)
- uavcan.dynid.fd (enable FD dynamic allocation)
- uavcan.param_set_behavior (SetRam / SetRamFlash / SetRamFlashAndReboot)
- bus.serial.baudrate (0..460800, where 0 enables GNSS UBX auto-baud probing)
- bus.i2c.frequency_khz
- ROLE.* toggles for each role

Parameter handling is implemented in:
- [microcan/source/UAVCanSlave.cpp](../../microcan/source/UAVCanSlave.cpp) (GetSet, ExecuteOpcode)
- [COMMON/source/deviceResource.cpp](../../COMMON/source/deviceResource.cpp) (storage instance)

Independent sensor activation parameters (all default false):
`ROLE.adc.microphone.im68a130`, `ROLE.i2c.light.opt4060`,
`ROLE.i2c.range.vl53l4cx`. Publication periods, OPT4060 address/interrupt mode
and the range sensor ID are documented in
[independent sensors](roles/independent_sensors.md). Save and reboot to apply.
