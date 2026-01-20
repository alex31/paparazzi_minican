# Onboard RGB LED Codes

The onboard WS2812 LED indicates status and errors. Implementation:
[COMMON/source/rgbLeds.cpp](../../COMMON/source/rgbLeds.cpp) and
[COMMON/source/rgbLeds.hpp](../../COMMON/source/rgbLeds.hpp).

## Pattern encoding
- The LED uses a 16-bit motif. Each step lasts period_ms from RgbLed::setMotif().
- Bit order: step 0 = bit0 (LSB), step 15 = bit15 (MSB).
- If the current bit is 1, the LED shows the current color; if 0, the LED is off.
- Priority order: wheel-of-death animation > node ID display > motif > off.

## Node ID display (base-5 colors)
The node ID is encoded as three base-5 digits:
id = d0 * 25 + d1 * 5 + d2, with digits in the range 0..4.

Digit colors:
- 0 = white
- 1 = red
- 2 = yellow
- 3 = green
- 4 = blue

Timing:
- Each digit: 0.5 s on, 0.3 s off
- After all 3 digits: 1 s off
- Node ID 0 is not displayed (digits are all zero, raw value = 0)
- During dynamic ID allocation, the startup motif remains active until an ID is
  assigned, then the node ID display takes over

## Application (minican) LED codes
Patterns set in [minican/source/main.cpp](../../minican/source/main.cpp):
- Boot/startup (before checks): greenish, motif 0b1010101010101010, period 500 ms
- Firmware/hardware mismatch: wheel-of-death (continuous rainbow sweep)
- MFS init failure: blue, motif 0b10101010, period 200 ms
- Persistent storage init failure: purple (dim), motif 0b10101010, period 200 ms
- Identification mode (ROLE.identification=true): purple (bright),
  motif 0b1010100000000000, period 150 ms
- CAN start failure (resource conflict): red, motif 0b110011000, period 100 ms
- CAN start failure (other): orange, motif 0b10101010, period 200 ms
- Normal operation: node ID display takes over once CAN starts

## Bootloader LED codes
Patterns set in [bootloader/source/main.cpp](../../bootloader/source/main.cpp)
using RgbLed::startMinimal():
- Bootloader active (including firmware update): blue,
  motif 0b1010100000000000, period 100 ms
- Application corrupted (post-flash CRC error): red,
  motif 0b1010101010101010, period 100 ms, infinite halt
- Header/protocol/size/address errors: yellow,
  motif 0b1010101010101010, period 100 ms, then sleep
  - MAGIC_ERROR_AGAIN: 2 s, then boot attempt
  - Other errors: 10 s, then boot attempt
