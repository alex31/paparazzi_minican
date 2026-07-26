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

`ImavRole` acquires PA3, ADC1 and TIM6 for its single 23.9977 kHz microphone
channel. It also starts the shared I2C1 infrastructure for the OPT4060 and
VL53L4CX on PA15/PB07. PA4 and ADC2 remain available to other roles.

PB07 is also TIM3_CH4. An IMAV configuration using I2C1 therefore cannot run
at the same time as a role that reassigns PB07 to the LED strip or voltmeter.
The role must be started before such optional roles so these conflicts are
reported deterministically.
