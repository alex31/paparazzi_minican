# Hardware Resource Allocation

Roles must acquire resources using boardResource.tryAcquire(...) to avoid
conflicts.

Typical resources include:

- Timers (TIM1, TIM3, TIM7)
- UARTs (USART2)
- I2C (I2C1)
- ADCs (ADC1 and ADC2)
- GPIO pins (PA08..PA11, PB07, etc.)

This is enforced in each role start() function to prevent multiple roles from
using the same pins or timers.

## IMAV role

`ImavRole` acquires PA4, PA8, ADC2 and TIM6. PA4/ADC2_IN17 and TIM6 implement
the single 23.9977 kHz microphone channel; PA8/EXTI8 receives the active-low
OPT4060 data-ready pulse formerly available as SRV1. ADC1 remains dedicated to
continuous voltage and core-temperature monitoring. The role also starts the
shared I2C1 infrastructure for the OPT4060 and, when
`role.imav.time_of_flight` is enabled, the optional VL53L4CX on PA15/PB07.

PA4 is also the external SPI chip select and TIM3_CH2. IMAV therefore conflicts
with any role that tries to use that pin for external SPI or PWM channel 6.
PA8 is dedicated to OPT4060_INT in this branch and is no longer a servo or
DShot output.

PB07 is also TIM3_CH4. An IMAV configuration using I2C1 therefore cannot run
at the same time as a role that reassigns PB07 to the LED strip or voltmeter.
The role must be started before such optional roles so these conflicts are
reported deterministically.
