# Software Overview

MiniCAN is a UAVCAN (DroneCAN) peripheral node based on an STM32G491 MCU. It
bridges the CAN network to physical peripherals (UART, I2C, PWM, DShot, LEDs)
via a modular role system.

Key properties:
- UAVCAN slave node with CAN FD support
- Battery input 2 to 6 cells, onboard 3.3V and 5V rails
- Bootloader with CRC and safe update process over UAVCAN
- Persistent parameter storage and UAVCAN GetSet support
- Modular roles enabled via persistent parameters
- Resource manager to prevent peripheral conflicts

Repository layout:
- [bootloader/](../../bootloader/) : Bootloader application
- [minican/](../../minican/) : Main firmware (UAVCAN node + roles)
- [COMMON/](../../COMMON/) : Shared drivers, roles, utilities
- [HARDWARE/](../../HARDWARE/) : Schematics and board files for MiniCAN/MicroCAN

For detailed roles, see [roles/overview.md](roles/overview.md) and
[roles.readme.txt](../../roles.readme.txt).
