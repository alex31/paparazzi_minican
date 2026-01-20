# Parameter System

Parameter list: [COMMON/source/nodeParameters.hpp](../../COMMON/source/nodeParameters.hpp).
Stored in external EEPROM via Persistant::Storage and exposed via UAVCAN GetSet.

Important parameters:
- uavcan.node_id (0 = dynamic ID)
- uavcan.dynid.fd (enable FD dynamic allocation)
- uavcan.param_set_behavior (SetRam / SetRamFlash / SetRamFlashAndReboot)
- bus.serial.baudrate
- bus.i2c.frequency_khz
- ROLE.* toggles for each role

Parameter handling is implemented in:
- [minican/source/UAVCanSlave.cpp](../../minican/source/UAVCanSlave.cpp) (GetSet, ExecuteOpcode)
- [COMMON/source/deviceResource.cpp](../../COMMON/source/deviceResource.cpp) (storage instance)
