# MiniCAN Project Documentation

This document provides a complete overview of the MiniCAN project: firmware, hardware, build
process, role system, and how to add new roles. It is written for the MiniCAN platform as the
primary target. A MicroCAN annex is included at the end.

Table of contents
- Overview
- Hardware overview (MiniCAN)
- Firmware architecture
- Build and flash
- Role system overview
- Roles (detailed)
- Parameter system
- Compilation toggles
- Hardware resource allocation
- How to add a new role
- How to add a new role with an LLM
- Annex: MicroCAN differences


## Overview

MiniCAN is a UAVCAN (DroneCAN) peripheral node based on an STM32G491 MCU. It bridges the CAN
network to physical peripherals (UART, I2C, PWM, DShot, LEDs) and publishes/consumes UAVCAN
messages through a modular role system.

Key properties:
- UAVCAN slave node with CAN FD support
- Bootloader in external SPI EEPROM (M95P), with CRC and safe update process
- Modular roles (servo control, ESC, GPS, sensors, serial tunnel, LED strip, voltmeter, etc.)
- Persistent parameter storage and UAVCAN param GetSet support

Repository layout:
- `bootloader/` : Bootloader application
- `minican/` : Main firmware (UAVCAN node + roles)
- `COMMON/` : Shared drivers, roles, utilities
- `HARDWARE/` : Schematics and board files for MiniCAN/MicroCAN


## Hardware overview (MiniCAN)

Board: MiniCAN v5 (STM32G491K, 170 MHz, single-precision FPU).

Core peripherals:
- CAN FD on FDCAN2 (PB05/PB06)
- UART: USART2 (PB03 TX, PB04 RX) used for GPS/serial tunnel/smart servo, etc.
- I2C: I2C1 (PA15 SCL, PB07 SDA)
- SPI1 for external M95P EEPROM (PA05/PA06/PA07 + PB00 CS)
- PWM: TIM1 channels on PA08..PA11 for servo/ESC DShot
- WS2812 LED strip output on TIM3_CH4 (PB07)
- Debug UART on LPUART1 (PA02/PA03)

Notable pins (MiniCAN):
- PB08: RGBLED (TIM8_CH2), also BOOT0
- PF01: CAN termination enable
- PA00: Battery sense ADC input (PS_5V)

Key board files:
- `minican/cfg/MINICAN.cfg` (pin map)
- `HARDWARE/MINICAN/*.pdf` (schematics)

Shared resources:
- TIM3_CH4 is used by both `ROLE.led2812` and `ROLE.voltmeter` (mutually exclusive).
- USART2 is used by GPS, serial tunnel, smart servos (roles must not conflict).


## Firmware architecture

The system is split into two firmware images:
- Bootloader (in internal flash) controls firmware update from external M95P EEPROM.
- Application (main firmware) runs the UAVCAN node and role system.

Core components:
- `UAVCanSlave` (minican/source/UAVCanSlave.cpp):
  - Creates the UAVCAN node
  - Handles UAVCAN services (GetNodeInfo, Param GetSet, Restart, FW update)
  - Dynamically instantiates enabled roles
- `COMMON/source/roleBase.hpp`:
  - Role interface and singleton/trampoline helpers
- `COMMON/source/resourceManager.hpp`:
  - Prevents peripheral/pin conflicts between roles
- `COMMON/source/MFS.*` and `mfsOnM95p.*`:
  - Persistent storage on external EEPROM
- `COMMON/source/firmwareUpdate.*` and `firmwareHeader.hpp`:
  - Firmware update protocol and header management


## Build and flash

Top-level build (bootloader + firmware for both MiniCAN/MicroCAN):
```sh
make
```

Build only MiniCAN firmware:
```sh
make minican
```

Build only MicroCAN firmware:
```sh
make microcan
```

Clean all:
```sh
make clean
```

Build arguments:
- `CAN_BITRATE=1000` (default): CAN FD enabled if > 1000
- `RELEASE=g` (debug), `RELEASE=fast` (speed), `RELEASE=s` (size)
- `PRECISE_FDCAN_TIMINGS=true` for tighter CAN timing

Example:
```sh
make clean
make CAN_BITRATE=1000 RELEASE=s
```

Flash (requires bmpflash or other configured flasher):
```sh
make flashmini
make flashmicro
```

Initial flashing via ROM UART bootloader:
```sh
make uartflashmini DEV=/dev/ttyUSB0
make uartflashmicro DEV=/dev/ttyUSB1
```

Bootloader size is set in top-level `Makefile` (see `BOOTLOADER_SIZE`).


## Role system overview

Roles are modular features enabled by parameters and compiled in/out by macros.
Each role:
- Inherits from `RoleBase`
- Implements `subscribe()` and `start()`
- Acquires hardware resources via `boardResource`

Role creation and activation flow:
1) Role compiled in if `USE_*_ROLE` in `COMMON/source/roleConf.h` is true.
2) Role instantiated at startup if its parameter `ROLE.*` is enabled.
3) Role subscribes to UAVCAN if needed.
4) Role starts and allocates its hardware resources.


## Roles (detailed)

Source reference: `roles.readme.txt` and `COMMON/source/*Role*`.

### ServoRole (`ROLE.servo.pwm`, `ROLE.servo.smart`)
Receives `uavcan.equipment.actuator.ArrayCommand` and routes to PWM or smart servo backend.

PWM params:
- `role.servo.pwm.frequency`
- `role.servo.pwm.pulse_half_width`
- `role.servo.pwm.map_index1`
- `role.servo.pwm.channel_mask`

Smart servo params:
- `role.servo.smart.map_index1`
- `role.servo.smart.num_servos`
- `role.servo.smart.status_frequency`

Outputs:
- `uavcan.equipment.actuator.Status` (smart servo status only)

Wiring:
- PWM on TIM1 CH1..CH4 (PA08..PA11)
- Smart servos use UART (USART2 on MiniCAN)

### EscDshot (`ROLE.esc.dshot`)
Receives `uavcan.equipment.esc.RawCommand` and drives DShot on TIM1 CH1..CH4.

Params:
- `role.esc.dshot.map_index1`
- `role.esc.dshot.channel_mask`
- `role.esc.dshot.cmd_rate` (ChibiOS ticks)
- `role.esc.dshot.rpm_freq_div` (0 disables telemetry)

Telemetry:
- `uavcan.equipment.esc.Status` if bidirectional DShot is enabled.

Wiring:
- TIM1 CH1..CH4 on PA08..PA11 (MiniCAN)

### SBUS RC (`ROLE.sbus`)
Reads SBUS on UART and publishes `dronecan.sensors.rc.RCInput`.

Params:
- `role.sbus.channel_mask`
- `role.sbus.id`

Wiring:
- SBUS on USART2 RX (PB04) on MiniCAN

### SerialStream (`ROLE.tunnel.serial`)
UART <-> `uavcan.tunnel.Broadcast` bridge.

Params:
- `role.tunnel.serial.protocol`
- `bus.serial.baudrate`

Wiring:
- USART2 TX/RX on PB03/PB04 (MiniCAN)

### GpsUBX (`ROLE.gnss.ubx`)
Decodes UBX from UART and publishes:
- `uavcan.equipment.gnss.Fix2` (NAV-PVT)
- `uavcan.equipment.gnss.Auxiliary` (DOP/SAT)

Params:
- `bus.serial.baudrate`

Notes:
- MiniCAN uses full-duplex UART and attempts GPS auto-configuration (UBX CFG/VALSET).
- MicroCAN is RX-only and expects GPS to be preconfigured.

Wiring:
- USART2 TX/RX on PB03/PB04 (MiniCAN)

### QMC5883 Magnetometer (`ROLE.i2c.magnetometer.q5883`)
Publishes `uavcan.equipment.ahrs.MagneticFieldStrength2`.

Params:
- `role.i2c.magnetometer.q5883.range` (2 or 8 gauss)
- `role.i2c.magnetometer.q5883.rot_deg` (0/90/180/270)
- `role.i2c.magnetometer.q5883.sensor_id`

Wiring:
- I2C1 on PA15 (SCL) + PB07 (SDA)

### MPL3115A2 Barometer (`ROLE.i2c.barometer.mpl3115a2`)
Publishes `uavcan.equipment.air_data.StaticPressure` and `StaticTemperature`.

Wiring:
- I2C1 on PA15 (SCL) + PB07 (SDA)

### HealthSurvey (`ROLE.health.survey`)
Publishes board health:
- `uavcan.equipment.device.Temperature`
- `uavcan.equipment.power.CircuitStatus` (battery + 3.3V)

### LED Strip (`ROLE.led2812`)
Receives `uavcan.equipment.indication.LightsCommand` to drive WS2812 strip.

Params:
- `role.led2812.led_number` (1..8)

Wiring:
- TIM3_CH4 on PB07 (shared with voltmeter)

### Voltmeter (`ROLE.voltmeter`)
Displays battery status on WS2812 LED strip. Can blank LEDs based on GPS speed.

Params:
- `role.voltmeter.cells`
- `role.voltmeter.brightness`
- `role.voltmeter.gps_speed_off_mps`

Wiring:
- TIM3_CH4 on PB07 (shared with LED strip)

### Onboard RGB LED (internal)
Used for node status/identification. Implemented in `COMMON/source/rgbLeds.*`.


## Parameter system

Parameter list: `COMMON/source/nodeParameters.hpp`.
Stored in external EEPROM via `Persistant::Storage` and exposed via UAVCAN GetSet.

Important parameters:
- `uavcan.node_id` (0 = dynamic ID)
- `uavcan.dynid.fd` (enable FD dynamic allocation)
- `uavcan.param_set_behavior` (SetRam / SetRamFlash / SetRamFlashAndReboot)
- `bus.serial.baudrate`
- `bus.i2c.frequency_khz`
- `ROLE.*` toggles for each role

Parameter handling is implemented in:
- `UAVCanSlave.cpp` (GetSet, ExecuteOpcode)
- `COMMON/source/deviceResource.*` (storage instance)


## Compilation toggles

Compile-time role toggles are in `COMMON/source/roleConf.h`:
- `USE_SERVO_ROLE`
- `USE_BARO_MPL3115A2_ROLE`
- `USE_QMC5883_ROLE`
- `USE_ESC_DSHOT_ROLE`
- `USE_RC_SBUS_ROLE`
- `USE_GPS_UBX_ROLE`
- `USE_SERIAL_STREAM_ROLE`
- `USE_LED2812_ROLE`
- `USE_VOLTMETER_ROLE`

These must be enabled for roles to be compiled in. Runtime enable is separate.


## Hardware resource allocation

Roles must acquire resources using `boardResource.tryAcquire(...)` to avoid conflicts.
Typical resources include:
- Timers (TIM1, TIM3, TIM7)
- UARTs (USART2)
- I2C (I2C1)
- GPIO pins (PA08..PA11, PB07, etc.)

This is enforced in each role `start()` function to prevent multiple roles
from using the same pins/timers.


## How to add a new role

The recommended starting point is the built-in template role:
`COMMON/source/templateRole.hpp` and `COMMON/source/templateRole.cpp`.
Copy these files, rename the class and files, then adapt subscriptions,
resources, and parameters to your new role.

1) Create role files
   - Copy `COMMON/source/templateRole.hpp` and `COMMON/source/templateRole.cpp`.
   - Rename to `COMMON/source/MyRole.hpp` and `COMMON/source/MyRole.cpp`.
   - Rename the class (`TemplateRole` -> `MyRole`) and update comments.

2) Add role parameter(s)
   - Add `ROLE.my_role` to `COMMON/source/nodeParameters.hpp`.
   - Add any runtime parameters (example in template: `role.template.log_every`).

3) Add compile-time toggle
   - Add `#define USE_MY_ROLE true` in `COMMON/source/roleConf.h`.

4) Register in UAVCanSlave
   - Add role class include in `minican/source/UAVCanSlave.cpp`.
   - Add `addRole<MyRole, FixedString("ROLE.my_role")>();` in `CANSlave::start()`.

5) Allocate resources
   - In `start()`, use `boardResource.tryAcquire(...)` for pins and peripherals.
   - If MicroCAN uses shared pins, use `DynPin::setScenario(...)` where needed.

6) Subscribe and publish
   - Implement `subscribe()` for UAVCAN message subscriptions.
   - Implement `start()` to launch threads, begin hardware, and publish messages.

7) Document the role
   - Add an entry in `roles.readme.txt`.
   - Update this document if the role is user-visible.


## How to add a new role with an LLM

Recommended workflow:
1) Provide the LLM with:
   - Desired role behavior (messages in/out, timing, hardware interface)
   - Pins/peripherals required
   - Parameter names, limits, defaults

2) Ask the LLM to:
   - Start from `COMMON/source/templateRole.hpp/.cpp`
   - Copy/rename the files and class
   - Add parameters to `nodeParameters.hpp`
   - Add compile-time toggle in `roleConf.h`
   - Register in `UAVCanSlave.cpp`
   - Add documentation to `roles.readme.txt`

3) Review output for:
   - Correct resource acquisition
   - Thread safety
   - Parameter naming conventions
   - Consistency with existing roles

Example prompt:
```
Create a new role named FooRole for MiniCAN.
It should read a sensor over I2C1 at 50 Hz and publish uavcan.equipment.foo.Bar.
Use parameter ROLE.foo to enable it and role.foo.sensor_id to set sensor ID.
Add resource allocation for I2C1 and PB07/PA15 pins.
Register it in UAVCanSlave and document it in roles.readme.txt.
```


## Annex: MicroCAN differences

MicroCAN is a smaller board with shared pin headers. It is likely to be deprecated,
but the firmware still supports it with specific constraints:

Hardware differences:
- Shared pins F0..F4 used for multiple functions
- UART/I2C/SPI/PWM are mutually exclusive on shared pins
- UART may be RX-only in some configurations

Firmware differences:
- `PLATFORM_MICROCAN` uses `dynamicPinConfig` to switch pin functions.
- Some roles are disabled or limited due to pin sharing.
- GPS auto-configuration is not used (RX-only UART).

Key files:
- `minican/cfg/MICROCAN.cfg`
- `COMMON/source/dynamicPinConfig.*`

If MicroCAN is removed in the future, `dynamicPinConfig` and related
resource conflicts can be simplified or removed for MiniCAN-only builds.
