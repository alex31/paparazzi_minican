# MiniCAN

MiniCAN is a flexible UAVCAN interface card based on an STM32G491 microcontroller. It acts as a proxy between a UAVCAN network and a variety of peripherals such as servos, ESCs, sensors and serial links. The project contains both the bootloader and the main firmware used on the board.

## Features

- **UAVCAN slave node** with CAN FD support
- **Bootloader** able to update the firmware from an external M95P SPI flash memory with CRC checks for reliability
- **Servo control**: standard PWM outputs and support for smart servos
- **DShot ESC driver** with optional bi‑directional telemetry
- **SBUS tunnel** to forward RC frames over UAVCAN
- **Telemetry tunnel** for Paparazzi messages or raw serial data
- **I2C sensor support** (e.g. MPL3115A2 barometer)
- **Health survey** publishing battery and core temperature measurements
- Configurable roles and parameters stored in persistent memory

Hardware design files for the MiniCAN and MicroCAN board are available under `HARDWARE/`.

## Repository layout

- `bootloader/` – bootloader source code and configuration
- `minican/` – main application implementing the UAVCAN node and all roles
- `COMMON/` – modules shared between bootloader and application
- `HARDWARE/` – hardware schematics and CubeMX configuration

## Building

A GNU Arm embedded toolchain and ChibiOS 21.11 are required. The Makefiles expect this repository to reside next to a `ChibiOS_21.11_stable` folder. Run:

```bash
make            # builds bootloader and firmware
make flash       # flashes both images using bmpflash
```

You can also build `bootloader` or `minican` individually by invoking `make -C bootloader` or `make -C minican`.

## License

This project includes ChibiOS components released under the Apache License 2.0. The rest of the code is provided without any specific license.
