# Software Overview

MicroCAN is a UAVCAN (DroneCAN) peripheral node based on an STM32G491 MCU. It
bridges the CAN network to physical peripherals (UART, I2C, ADC, PWM, DShot,
LEDs and sensors) via a modular role system.

Key properties:
- UAVCAN slave node with CAN FD support
- Battery input 2 to 6 cells, onboard 3.3V and 5V rails
- Bootloader with CRC and safe update process over UAVCAN
- Persistent parameter storage and UAVCAN GetSet support
- Modular roles enabled via persistent parameters
- Resource manager to prevent peripheral conflicts
- Optional IMAV beacon role using one analog microphone and an OPT4060 color
  sensor, with optional VL53L4CX ground ranging

Repository layout:
- [bootloader/](../../bootloader/) : Bootloader application
- [microcan/](../../microcan/) : Main firmware (UAVCAN node + roles)
- [COMMON/](../../COMMON/) : Shared drivers, roles, utilities
- [HARDWARE/](../../HARDWARE/) : Schematics and board files

For detailed roles, see [roles/overview.md](roles/overview.md) and
[roles.readme.txt](../../roles.readme.txt).

The IMAV hardware discussion and current implementation details are in
[IMAV/sensors_electronic.md](../../IMAV/sensors_electronic.md) and
[IMAV/sensors.md](../../IMAV/sensors.md).
