#pragma once

      {"can.terminal_resistor", {.v = true}},
      {"can.bit_rate", {.min = 100'000, .max = 8'000'000, .v = 1'000'000}},
	
      {"uavcan.node_id", {.min = 8, .max = 127, .v = 127}},
      {"uavcan.last_msg", {.v = "just flashed"}},
	
      {"bus.i2c.pullup_resistor", {.v = true}},

      {"hardware.type", {.min = Type_minicanV5, .max =  Type_unknown, .v = Type_unknown}},
      // fancy led pattern if role.identification is true
      {"role.identification", {.v = true}},
	
      {"role.servo.pwm", {.v = false}},
      {"role.servo.pwm.frequency", {.min=50, .max=560, .v = 50}},
      {"role.servo.pwm.pulse_half_width", {.v = false}},
      {"role.servo.pwm.map_index1", {.min=0, .max=124, .v = 0}},
      {"role.servo.pwm.num_servos", {.min=1, .max=4, .v = 4}},
	
      {"role.servo.smart", {.v = false}},
      {"role.servo.smart.map_index1", {.min=0, .max=124, .v = 0}},
      {"role.servo.smart.num_servos",  {.min=1, .max=8, .v = 1}},

      {"role.esc.dshot", {.v = false}},
      {"role.esc.dshot.map_index1", {.min=0, .max=19, .v = 0}},
      {"role.esc.dshot.num_channels",  {.min=1, .max=4, .v = 1}},
      {"role.esc.dshot.cmd_rate",  {.min=100, .max=1000, .v = 100}},
      {"role.esc.dshot.rpm_freq_div",  {.min=0, .max=1000, .v = 0}}, // 0 disable bidir telemetry


      {"role.tunnel.sbus", {.v = false}},
      // envoyer toutes les trames sbus sur le can occuperait 8% de la BP, il faut optimiser
      // on n'envoie que les cannaux actifs (1 actif, 0 inactif)
      {"role.tunnel.sbus.active_channels", {.v = 0x1111111111111111}},

      {"role.tunnel.telemetry", {.v = false}},
      // if xbee_frame == false, pprz_frame will be used
      {"role.tunnel.telemetry.xbee_frame", {.v = false}},
      {"role.tunnel.telemetry.baudrate", {.min = 1'200, .max = 460'400, .v = 57'600}},

      {"role.i2c.barometer.mpl3115a2", {.v = false}},
	
      {"dbg.ratio", {.min = -1.0f, .max = 1.0f, .v = 0.5f}},




	
