# MicroCAN v3

MicroCAN is a smaller board with shared pin headers. The firmware supports it
with specific constraints.

Hardware differences:
- Shared pins F0..F4 used for multiple functions
- UART/I2C/SPI/PWM are mutually exclusive on shared pins
- UART may be RX-only in some configurations

Firmware differences:
- PLATFORM_MICROCAN uses dynamicPinConfig to switch pin functions
- Some roles are disabled or limited due to pin sharing
- GPS auto-configuration is not used (RX-only UART)

Key files:
- [minican/cfg/MICROCAN.cfg](../../../minican/cfg/MICROCAN.cfg)
- [COMMON/source/dynamicPinConfig.cpp](../../../COMMON/source/dynamicPinConfig.cpp)
- [HARDWARE/MICROCAN/](../../../HARDWARE/MICROCAN/)
