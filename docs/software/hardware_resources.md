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

## Shared I2C startup

I2C roles explicitly call `I2CPeriph::start()`. The first call reserves I2C1,
PA15 and PB07; later callers reuse the running bus. Do not acquire these
resources separately in a sensor role. Startup is sequential and errors are
propagated by each role; dependencies are not resolved automatically.
Transactions and recovery share the I2C mutex (`resetLocked()` for its owner,
`reset()` otherwise).

The [independent sensor roles](roles/independent_sensors.md) add:

| Role | Exclusive resources | Shared resources |
| --- | --- | --- |
| Microphone IM68A130 | PA4, ADC2, TIM6, DMA stream | None |
| OPT4060 | PA8 when interrupts are enabled | I2C1 |
| VL53L4CX | None | I2C1 |

ADC1 remains dedicated to health monitoring. PA4 conflicts with PWM CH6;
PA8 conflicts with PWM CH1/DShot; PB07 conflicts with PWM CH7 and WS2812 roles.
