# MiniCAN v5

Board: MiniCAN v5 (STM32G491K, 170 MHz, single-precision FPU).

Core peripherals:
- CAN FD with software selectable terminal resistor
- UART for GPS, serial tunnel, smart servo, etc.
- I2C for external peripherals, software selectable pullups
- SPI shared between internal M95P EEPROM and external peripherals
- TIM/PWM for servo, DShot, RGB LED strip
- Onboard RGB LED for node ID and status
- Debug pads with SWD and UART
- ADC monitoring for battery and 5V

Key files:
- [minican/cfg/MINICAN.cfg](../../../minican/cfg/MINICAN.cfg) (pin map)
- [HARDWARE/MINICAN/](../../../HARDWARE/MINICAN/) (schematics)
