# Firmware Architecture

The system is split into two firmware images:
- Bootloader controls the second stage update from external M95P EEPROM.
- Application (main firmware) runs the UAVCAN node, role system, and the first
  stage firmware update protocol.

Core components:
- [microcan/source/UAVCanSlave.cpp](../../microcan/source/UAVCanSlave.cpp)
  - Creates the UAVCAN node
  - Handles UAVCAN services (GetNodeInfo, Param GetSet, Restart, FW update)
  - Dynamically instantiates enabled roles
- [COMMON/source/roleBase.hpp](../../COMMON/source/roleBase.hpp)
  - Role interface and singleton/trampoline helpers
- [COMMON/source/resourceManager.hpp](../../COMMON/source/resourceManager.hpp)
  - Prevents peripheral/pin conflicts between roles
- [COMMON/source/MFS.cpp](../../COMMON/source/MFS.cpp) and
  [COMMON/source/mfsOnM95p.c](../../COMMON/source/mfsOnM95p.c)
  - Persistent storage on external EEPROM
- [COMMON/source/firmwareUpdate.cpp](../../COMMON/source/firmwareUpdate.cpp) and
  [COMMON/source/firmwareHeader.hpp](../../COMMON/source/firmwareHeader.hpp)
  - Firmware update protocol and header management

Enabled roles are allocated dynamically during `CANSlave::start()`. Large
role-specific buffers, sensor contexts and thread working areas are allocated
only from each role's `start()` method. Read-only tables may remain in Flash;
large mutable objects must not consume static RAM while their role is disabled.
