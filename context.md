• Here are the key locations we’ve been using:

  - Project sources (role code): `COMMON/source`
      - Example: serialStreamRole.cpp/.hpp, gpsUbxRole.cpp, etc.
  - Project build dir: `microcan`
  - Shared utilities (various): /home/alex/DEV/STM32/CHIBIOS/COMMON/various
      - stdutil.h, stdutil.c, fifoObject.hpp, etc.
  - ChibiOS core: /home/alex/DEV/STM32/CHIBIOS/ChibiOS_21.11_stable
  - DSDL definition : /home/alex/DEV/STM32/UAVCAN/DSDL and /home/alex/DEV/STM32/UAVCAN/DSDLC
  - external library (frozen, etl, ...) : /home/alex/DEV/STM32

• Recent achievements (branch: feature/use_sio):
  - Validated the migration of serial roles from USART to SIO driver. No regressions found.
  - Refactored `ServoSmartRole`, `VoltmeterRole`, and `RgbLedRole` to use dynamic heap allocation (`new`) for heavy static components (`Led2812Strip`, `STS3032`, `SIO::Datagram`) instead of static BSS placement. This strictly adheres to the rule that roles must only consume memory if active, saving ~1.2 KB of permanent BSS memory.
  - Resolved `std::visit` compilation errors in `ttyConsole.cpp`.
