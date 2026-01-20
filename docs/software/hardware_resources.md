# Hardware Resource Allocation

Roles must acquire resources using boardResource.tryAcquire(...) to avoid
conflicts.

Typical resources include:
- Timers (TIM1, TIM3, TIM7)
- UARTs (USART2)
- I2C (I2C1)
- GPIO pins (PA08..PA11, PB07, etc.)

This is enforced in each role start() function to prevent multiple roles from
using the same pins or timers.
