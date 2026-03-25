# Build and Flash

Top level build (bootloader + MicroCAN firmware):
```sh
make
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
- CAN_BITRATE=1000 (default). CAN FD enabled if > 1000.
- RELEASE=g (debug), RELEASE=fast (speed), RELEASE=s (size)
- PRECISE_FDCAN_TIMINGS=true for tighter CAN timing

Example:
```sh
make clean
make CAN_BITRATE=1000 RELEASE=s
```

Flash (requires bmpflash or other configured flasher):
```sh
make flash
make flashmicro
```

Initial flashing via ROM UART bootloader:
```sh
make uartflashmicro DEV=/dev/ttyUSB0
```

## First flash over the STM32 ROM UART bootloader

This section is for the very first flash of a blank board, using a USB to
serial adapter. It uses the STM32G491 ROM bootloader, not the normal
application UART.

### Important: use `SRV2` and `SRV3`, not `UART_TX` and `UART_RX`

Do **not** use the board `UART_TX` / `UART_RX` connector (`PB03` / `PB04`,
USART2) for ROM bootloader flashing. That UART is used by the application, but
it is not the convenient bootloader path described here.

Use the servo connector pins tied to `USART1` instead:
- `SRV2` = `PA09` = MCU `USART1_TX`
- `SRV3` = `PA10` = MCU `USART1_RX`

### What you need

- A USB to serial adapter with **3.3 V TTL logic**
- Three wires
- A way to force `BOOT0` high during reset

Do **not** use an RS-232 adapter. Do **not** feed a 5 V serial signal into the
STM32 pins.

### Wiring

Connect the USB to serial adapter to the board like this:

- Adapter `TX` -> board `SRV3` (`PA10`, MCU RX)
- Adapter `RX` -> board `SRV2` (`PA09`, MCU TX)
- Adapter `GND` -> board `GND`

Notes:
- `TX` and `RX` are crossed on purpose.
- The servo connector also carries power rails, but for flashing you usually
  only need `TX`, `RX`, and `GND`.
- The safest setup is to power the board separately and only share `GND` with
  the USB to serial adapter.

### Enter ROM bootloader mode

1. Power the board off.
2. Force `BOOT0` high.
3. Power the board on, or reset it while `BOOT0` is high.
4. Keep the USB to serial adapter connected to `SRV2` / `SRV3`.

On this board, `BOOT0` is exposed as a PCB pad near the RGB LED and the
mounting ear hole, on the PCB side where the two CAN connectors are soldered.
In the pin map it is associated with `PB8` (`RGBLED` / `BOOT0`).

### Flash command

From the repository root:

```sh
make uartflashmicro DEV=/dev/ttyUSB0
```

If your adapter appears as another device, use that instead, for example:

```sh
make uartflashmicro DEV=/dev/ttyACM0
```

The Makefile uses STM32CubeProgrammer over UART with:
- `115200` baud
- `8E1` serial format (`8` data bits, `EVEN` parity, `1` stop bit)
- no flow control

### After flashing

1. Remove the `BOOT0` high strap.
2. Reset or power-cycle the board.
3. The board should now boot the flashed firmware normally.

### Troubleshooting

- If STM32CubeProgrammer says it cannot open `/dev/ttyUSB0`, the Linux device
  name is wrong or the port is already in use.
- If it opens the port but does not talk to the MCU, check `TX/RX` crossing,
  common `GND`, and that the adapter is really **3.3 V TTL**.
- If nothing answers, double-check that the board was reset with `BOOT0` high.
- If you are connected to `PB03` / `PB04` (`UART_TX` / `UART_RX`) instead of
  `SRV2` / `SRV3`, the ROM bootloader method above will not work.

Bootloader size is set in the top level Makefile (see BOOTLOADER_SIZE).
Generated *.uavcan.bin images are suitable for UAVCAN firmware update.
