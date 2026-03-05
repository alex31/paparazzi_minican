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

Bootloader size is set in the top level Makefile (see BOOTLOADER_SIZE).
Generated *.uavcan.bin images are suitable for UAVCAN firmware update.
