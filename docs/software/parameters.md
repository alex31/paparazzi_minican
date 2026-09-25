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

`ROLE.identification` defaults to true. At boot it selects the purple LED motif
and starts UAVCAN in MAINTENANCE mode without starting application roles,
including health survey. The shell still starts if `ROLE.shell=true`.
Node discovery/dynamic ID allocation, parameter access,
save/erase, restart and firmware updates remain available. Set it to false, save
and restart through DroneCAN GUI to enable normal role startup. Changes to this
mode take effect only on restart.

Identification is a boot-time override, not a configuration reset. All compiled
roles remain listed in the parameter interface, and their enable flags and
settings are preserved. Except for the shell, roles whose `ROLE.*` flag is true
are not started.
Clearing only `ROLE.identification`, saving and restarting restores normal
startup with those same settings; no role needs to be reconfigured.

Parameter handling is implemented in:
- [microcan/source/UAVCanSlave.cpp](../../microcan/source/UAVCanSlave.cpp) (GetSet, ExecuteOpcode)
- [COMMON/source/deviceResource.cpp](../../COMMON/source/deviceResource.cpp) (storage instance)

Independent sensor activation parameters (all default false):
`ROLE.adc.microphone.im68a130`, `ROLE.i2c.light.opt4060`,
`ROLE.i2c.range.vl53l4cx`. Publication periods, OPT4060 address/interrupt mode
and the range sensor ID are documented in
[independent sensors](roles/independent_sensors.md). Save and reboot to apply.

`ROLE.shell` (default false) enables the J3 diagnostic UART at 115200 bit/s,
8N1. Save and restart to apply. Identification preserves shell activation:
it starts only if `ROLE.shell=true`, in either mode. `NOSHELL=1`
builds omit this parameter and the shell. See [shell role](roles/shell.md).
