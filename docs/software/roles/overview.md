# Role System and Roles

Role system overview:
- Roles are modular features enabled by parameters.
- Roles are compiled in if USE_*_ROLE in
  [COMMON/source/roleConf.h](../../../COMMON/source/roleConf.h) is true.
- Roles are instantiated at startup if their ROLE.* parameter is enabled.
- Each role inherits RoleBase and implements subscribe() and start().
- Hardware resources are acquired via boardResource.tryAcquire(...).

Source reference: [roles.readme.txt](../../../roles.readme.txt) and
[COMMON/source/](../../../COMMON/source/) (files named *Role*).

## Roles (summary)

### ServoRole (ROLE.servo.pwm, ROLE.servo.smart)
Receives uavcan.equipment.actuator.ArrayCommand and routes to PWM or smart servo
backend.

PWM params:
- role.servo.pwm.ch1-4.frequency
- role.servo.pwm.ch1-4.shot125
- role.servo.pwm.ch5-7.frequency
- role.servo.pwm.ch5-7.shot125
- role.servo.pwm.map_index1
- role.servo.pwm.channel_mask
  Bit mapping: bit0=PA08, bit1=PA09, bit2=PA10, bit3=PA11, bit4=PB04, bit5=PA04, bit6=PB07.

Smart servo params:
- role.servo.smart.map_index1
- role.servo.smart.num_servos
- role.servo.smart.status_frequency

Outputs:
- uavcan.equipment.actuator.Status (smart servo status only)

Wiring:
- PWM CH1..CH4 on TIM1 (PA08..PA11)
- PWM CH5..CH7 on TIM3 (PB04, PA04, PB07)
- Smart servos use UART (USART2 on MicroCAN)

Resource notes:
- Enabling any of CH5..CH7 reserves TIM3 for ServoPWM.
- CH5 on PB04 conflicts with USART2 RX (`UART_RX`).
- CH6 on PA04 conflicts with SPI peripheral chip select (`SPI_PERIPH_CS`).
- CH7 on PB07 conflicts with I2C1 SDA and with roles already documented on TIM3_CH4
  (LED strip, voltmeter).
- If only CH1..CH4 are enabled, ServoPWM stays on TIM1 and does not reserve TIM3.

### EscDshot (ROLE.esc.dshot)
Receives uavcan.equipment.esc.RawCommand and drives DShot on TIM1 CH1..CH4.

Params:
- role.esc.dshot.map_index1
- role.esc.dshot.channel_mask
- role.esc.dshot.cmd_rate (ChibiOS ticks)
- role.esc.dshot.rpm_freq_div (0 disables telemetry)

Telemetry:
- uavcan.equipment.esc.Status if bidirectional DShot is enabled.

Wiring:
- TIM1 CH1..CH4 on PA08..PA11 (MicroCAN)

### SBUS RC (ROLE.sbus)
Reads SBUS on UART and publishes dronecan.sensors.rc.RCInput.

Params:
- role.sbus.channel_mask
- role.sbus.id
- role.sbus.debug_uart_ttl (true: non-inverted TTL debug input, false: normal SBUS inversion)

Wiring:
- SBUS on USART2 RX (PB04) on MicroCAN

### SerialStream (ROLE.tunnel.serial)
UART <-> uavcan.tunnel.Broadcast bridge.

Params:
- role.tunnel.serial.protocol
- bus.serial.baudrate

Wiring:
- USART2 TX/RX on PB03/PB04 (MicroCAN)

### GpsUBX (ROLE.gnss.ubx)
Decodes UBX from UART and publishes:
- uavcan.equipment.gnss.Fix2 (NAV-PVT)
- uavcan.equipment.gnss.Auxiliary (DOP/SAT)

Params:
- bus.serial.baudrate

Notes:
- `bus.serial.baudrate > 0`: fixed UART baudrate.
- `bus.serial.baudrate = 0`: GNSS UBX auto-baud probing on `57600`, `115200`, `230400`.
- Detect baud, then configure GPS while keeping that detected baud.

Wiring:
- USART2 TX/RX on PB03/PB04 (MicroCAN)

### QMC5883 Magnetometer (ROLE.i2c.magnetometer.q5883)
Publishes uavcan.equipment.ahrs.MagneticFieldStrength2.

Params:
- role.i2c.magnetometer.q5883.range (2 or 8 gauss)
- role.i2c.magnetometer.q5883.rot_deg (0/90/180/270)
- role.i2c.magnetometer.q5883.sensor_id

Wiring:
- I2C1 on PA15 (SCL) + PB07 (SDA)

### MPL3115A2 Barometer (ROLE.i2c.barometer.mpl3115a2)
Publishes uavcan.equipment.air_data.StaticPressure and StaticTemperature.

Wiring:
- I2C1 on PA15 (SCL) + PB07 (SDA)

### HealthSurvey (ROLE.health.survey)
Publishes board health:
- uavcan.equipment.device.Temperature
- uavcan.equipment.power.CircuitStatus (battery + 3.3V)

### LED Strip (ROLE.led2812)
Receives uavcan.equipment.indication.LightsCommand to drive WS2812 strip.

Params:
- role.led2812.led_number (1..8)

Wiring:
- TIM3_CH4 on PB07 (shared with voltmeter)

### Voltmeter (ROLE.voltmeter)
Displays battery status on WS2812 LED strip. Can blank LEDs based on GPS speed.

Params:
- role.voltmeter.cells
- role.voltmeter.brightness
- role.voltmeter.gps_speed_off_mps

Wiring:
- TIM3_CH4 on PB07 (shared with LED strip)

### TemplateRole (ROLE.template)
Minimal example role for developers. Subscribes to uavcan.protocol.NodeStatus and
optionally logs every N messages.

Params:
- role.template.log_every (0 disables logging)

See [docs/software/adding_roles.md](../adding_roles.md) for the full new role checklist.
