# Firmware Architecture

The system is split into two firmware images:
- Bootloader controls the second stage update from external M95P EEPROM.
- Application (main firmware) runs the UAVCAN node, role system, and the first
  stage firmware update protocol.

Core components:
- [minican/source/UAVCanSlave.cpp](../../minican/source/UAVCanSlave.cpp)
  - Creates the UAVCAN node
  - Handles UAVCAN services (GetNodeInfo, Param GetSet, Restart, FW update)
  - Dynamically instantiates enabled roles
- [COMMON/source/roleBase.hpp](../../COMMON/source/roleBase.hpp)
  - Role interface and singleton/trampoline helpers
- [COMMON/source/ressourceManager.hpp](../../COMMON/source/ressourceManager.hpp)
  - Prevents peripheral/pin conflicts between roles
- [COMMON/source/MFS.cpp](../../COMMON/source/MFS.cpp) and
  [COMMON/source/mfsOnM95p.c](../../COMMON/source/mfsOnM95p.c)
  - Persistent storage on external EEPROM
- [COMMON/source/firmwareUpdate.cpp](../../COMMON/source/firmwareUpdate.cpp) and
  [COMMON/source/firmwareHeader.hpp](../../COMMON/source/firmwareHeader.hpp)
  - Firmware update protocol and header management
