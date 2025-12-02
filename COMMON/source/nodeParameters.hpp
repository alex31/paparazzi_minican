#pragma once
#include "projectconf.h"
// roles are enabled with boolean setting with ROLE uppercase to make them visible in GUI or TUI
      {"can.terminal_resistor", {.v = true}},
	
      {"uavcan.node_id", {.min = -124, .max = 124, .v = 0}}, // 0 is dynamic, negative is prefered dynamic
      {"uavcan.dynid.fd", {.v = false}},
	
      {"bus.i2c.pullup_resistor", {.v = true}},
      // I²C frequency : valid values : 100, 400, 1000
      {"bus.i2c.frequency_khz", {.min = 100, .max = 1000, .v = 400}},
      {"hardware.nickname", {.v = "nickname"}},
      // fancy led pattern if role.identification is true
      {"ROLE.identification", {.v = true}},
	
      {"ROLE.health.survey", {.v = true}},

#if USE_SERVO_ROLE
      {"ROLE.servo.pwm", {.v = false}},
      {"role.servo.pwm.frequency", {.min=50, .max=560, .v = 50}},
      {"role.servo.pwm.pulse_half_width", {.v = false}},
      {"role.servo.pwm.map_index1", {.min=0, .max=124, .v = 0}},
      // when pins are shared between timer and other peripherals, select
      // the channels that are in use, other will be free
      // mask is : msb F4 F3 F2 F1 lsb
      {"role.servo.pwm.channel_mask", {.min=0, .max=0b1111, .v = 0b1111}},
	
      {"ROLE.servo.smart", {.v = false}},
      {"role.servo.smart.map_index1", {.min=0, .max=124, .v = 0}},
      {"role.servo.smart.num_servos",  {.min=1, .max=8, .v = 1}},
      {"role.servo.smart.status_frequency",  {.min=0, .max=100, .v = 0}}, // in Hz, 0 is no reporting
#endif

#if USE_ESC_DSHOT_ROLE
      {"ROLE.esc.dshot", {.v = false}},
      {"role.esc.dshot.map_index1", {.min=0, .max=19, .v = 0}},
      {"role.esc.dshot.num_channels",  {.min=1, .max=4, .v = 1}}, // FIXME : use mask as servoPwm instead of num_channels
      {"role.esc.dshot.cmd_rate",  {.min=100, .max=1000, .v = 100}},
      {"role.esc.dshot.rpm_freq_div",  {.min=0, .max=1000, .v = 0}}, // 0 disable bidir telemetry
#endif

#if USE_GPS_UBX_ROLE
      {"ROLE.gnss.ubx", {.v = false}},
      {"role.gnss.ubx.baudrate", {.min = 57'600, .max = 460'800, .v = 115'200}},
#endif

#if USE_RC_SBUS_ROLE
      {"ROLE.sbus", {.v = false}},
      // Sending all SBUS frames over CAN would consume 8% of the bandwidth; this must be optimized.
      // Only the active channels are sent (1 = active, 0 = inactive).
      {"role.sbus.channel_mask", {.min=0, .max=0b1111'1111'1111'1111, .v = 0b1111'1111'1111'1111}},
      // in case there is multiple RC, each RC must have a unique ID
      {"role.sbus.id", {.min=0, .max=0xff, .v = 0}},
      {"role.sbus.range.min", {.min=0, .max=2047, .v = 172}},
      {"role.sbus.range.max", {.min=0, .max=2047, .v = 1811}},
#endif

#if USE_PPRZLINK_TUNNEL_ROLE
      {"ROLE.tunnel.telemetry", {.v = false}},
      // if xbee_frame == false, pprz_frame will be used
      {"role.tunnel.telemetry.xbee_frame", {.v = false}},
      {"role.tunnel.telemetry.baudrate", {.min = 1'200, .max = 460'800, .v = 57'600}},
#endif

#if USE_BARO_MPL3115A2_ROLE
      {"ROLE.i2c.barometer.mpl3115a2", {.v = false}},
#endif

#if USE_QMC5883_ROLE
      // driver fixes ODR=50Hz and oversampling=256, continuous.
      // range is the only configurable parameter
      {"ROLE.i2c.magnetometer.q5883", {.v = false}},
      {"role.i2c.magnetometer.q5883.range", {.min = 2, .max = 8, .v = 2}},
      {"role.i2c.magnetometer.q5883.rot_deg", {.min = 0, .max = 270, .v = 0}},
      {"role.i2c.magnetometer.q5883.sensor_id", {.min = 0, .max = 250, .v = 0}},
#endif
