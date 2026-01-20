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
- role.servo.pwm.frequency
- role.servo.pwm.pulse_half_width
- role.servo.pwm.map_index1
- role.servo.pwm.channel_mask

Smart servo params:
- role.servo.smart.map_index1
- role.servo.smart.num_servos
- role.servo.smart.status_frequency

Outputs:
- uavcan.equipment.actuator.Status (smart servo status only)

Wiring:
- PWM on TIM1 CH1..CH4 (PA08..PA11)
- Smart servos use UART (USART2 on MiniCAN)

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
- TIM1 CH1..CH4 on PA08..PA11 (MiniCAN)

### SBUS RC (ROLE.sbus)
Reads SBUS on UART and publishes dronecan.sensors.rc.RCInput.

Params:
- role.sbus.channel_mask
- role.sbus.id

Wiring:
- SBUS on USART2 RX (PB04) on MiniCAN

### SerialStream (ROLE.tunnel.serial)
UART <-> uavcan.tunnel.Broadcast bridge.

Params:
- role.tunnel.serial.protocol
- bus.serial.baudrate

Wiring:
- USART2 TX/RX on PB03/PB04 (MiniCAN)

### GpsUBX (ROLE.gnss.ubx)
Decodes UBX from UART and publishes:
- uavcan.equipment.gnss.Fix2 (NAV-PVT)
- uavcan.equipment.gnss.Auxiliary (DOP/SAT)

Params:
- bus.serial.baudrate

Notes:
- MiniCAN uses full duplex UART and attempts GPS auto-configuration.
- MicroCAN is RX-only and expects GPS to be preconfigured.

Wiring:
- USART2 TX/RX on PB03/PB04 (MiniCAN)

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
