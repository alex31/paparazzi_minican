#pragma once
#include "roleConf.h"
/**
 * @file nodeParameters.hpp
 * @brief Parameter definition list used by the persistent parameter system.
 *
 * This header is intentionally a bare initializer list that is #included inside
 * the constexpr params_list definition (see UAVCAN/persistantParam.cpp). Each
 * entry becomes a Persistant::Parameter (name + bounds + default), and the
 * resulting indices/names are the ones exposed via uavcan.protocol.param.GetSet.
 * Roles are enabled with boolean settings using uppercase ROLE to make them
 * visible in GUI or TUI.
 */
      {"can.terminal_resistor", {.v = true}},
	
      {"uavcan.node_id", {.min = -124, .max = 124, .v = 0}}, // 0 is dynamic, negative is prefered dynamic
      {"uavcan.dynid.fd", {.v = false}},
      {"uavcan.param_set_behavior", {.min = SetRam, .max = SetRamFlashAndReboot, .v = SetRamFlash}},
	
      {"bus.i2c.pullup_resistor", {.v = true}},
      // I²C frequency : valid values : 100, 400, 1000
      {"bus.i2c.frequency_khz", {.min = 100, .max = 1000, .v = 400}},
      // shared baudrate for all roles that use usart (but smart servo which use autobaud)
      // 0 is reserved for ROLE.gnss.ubx to trigger UBX auto-baud probing
      // and GPS reconfiguration over UBX on startup.
      {"bus.serial.baudrate", {.min = 0, .max = 460'800, .v = 115'200}},
      {"hardware.nickname", {.v = "nickname"}},
      // fancy led pattern if role.identification is true
      {"ROLE.identification", {.v = true}},
	
      {"ROLE.health.survey", {.v = true}},
      //      {"role.health.survey.periodms", {.min = 100, .max = 10'000, .v = 1'000}},

#if USE_SERVO_ROLE
      {"ROLE.servo.pwm", {.v = false}},
      {"role.servo.pwm.ch1-4.frequency", {.min=50, .max=1000, .v = 50}},
      {"role.servo.pwm.ch1-4.shot125", {.v = false}},
      {"role.servo.pwm.ch5-7.frequency", {.min=50, .max=1000, .v = 50}},
      {"role.servo.pwm.ch5-7.shot125", {.v = false}},
      {"role.servo.pwm.map_index1", {.min=0, .max=124, .v = 0}},
      // when pins are shared between timer and other peripherals, select
      // the channels that are in use, other will be free
      // bit0=PA08, bit1=PA09, bit2=PA10, bit3=PA11, bit4=PB04, bit5=PA04, bit6=PB07
      {"role.servo.pwm.channel_mask", {.min=0b0000001, .max=0b1111111, .v = 0b0001111}},
	
      {"ROLE.servo.smart", {.v = false}},
      {"role.servo.smart.map_index1", {.min=0, .max=124, .v = 0}},
      {"role.servo.smart.num_servos",  {.min=1, .max=8, .v = 1}},
      {"role.servo.smart.status_frequency",  {.min=0, .max=100, .v = 0}}, // in Hz, 0 is no reporting
#endif

#if USE_ESC_DSHOT_ROLE
      {"ROLE.esc.dshot", {.v = false}},
      {"role.esc.dshot.map_index1", {.min=0, .max=19, .v = 0}},
      // Mask is: msb CH4 CH3 CH2 CH1 lsb.
      {"role.esc.dshot.channel_mask", {.min=0b0001, .max=0b1111, .v = 0b0001}},
      {"role.esc.dshot.cmd_rate",  {.min=100, .max=1000, .v = 100}},
      {"role.esc.dshot.rpm_freq_div",  {.min=0, .max=1000, .v = 0}}, // 0 disable bidir telemetry
#endif

#if USE_GPS_UBX_ROLE
      {"ROLE.gnss.ubx", {.v = false}},
#endif

#if USE_RC_SBUS_ROLE
      {"ROLE.sbus", {.v = false}},
      // Sending all SBUS frames over CAN would consume 8% of the bandwidth; this must be optimized.
      // Only the active channels are sent (1 = active, 0 = inactive).
      {"role.sbus.channel_mask", {.min=0, .max=0b1111'1111'1111'1111, .v = 0b1111'1111'1111'1111}},
      // in case there is multiple RC, each RC must have a unique ID
      {"role.sbus.id", {.min=0, .max=0xff, .v = 0}},
      // true: UART-level non-inverted signal (debug with USB-UART dongle)
      // false: real SBUS signal, inversion handled by USART peripheral.
      {"role.sbus.debug_uart_ttl", {.v = false}},
#endif

#if USE_SERIAL_STREAM_ROLE
      {"ROLE.tunnel.serial", {.v = false}},
      {"role.tunnel.serial.protocol", {.min = 0, .max = 255, .v = 0}},
      {"role.tunnel.serial.channel_id", {.min = 0, .max = 255, .v = 0}},
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

#if USE_LED2812_ROLE
      {"ROLE.led2812", {.v = false}},	
      {"role.led2812.led_number", {.min = 1, .max = 8, .v = 8}},	
#endif

#if USE_TEMPLATE_ROLE
      {"ROLE.template", {.v = false}},
      {"role.template.log_every", {.min = 0, .max = 1000, .v = 0}},
#endif

#if USE_IMAV_ROLE
      {"ROLE.imav.beacon", {.v = false}},
      // The manual gives 2.0--3.0 kHz and a demonstration unit was measured
      // near 2.43 kHz, below the 2.6--3.0 kHz band quoted by the rulebook.
      // Read when the role starts; reboot after changing it at runtime.
      {"role.imav.audio.band_low_hz", {.min = 2000, .max = 2600, .v = 2000}},
      // Weight of the newest recognized burst in the reported audio SNR IIR.
      // 1 disables smoothing; the value is read live for every completed burst.
      {"role.imav.audio.snr_alpha", {.min = 0.5f, .max = 1.0f, .v = 1.0f}},
      // OPT4060 address selected by ADDR: GND=0x44, VDD=0x45,
      // SDA=0x46, SCL=0x47. Invalid persisted values fall back to probing.
      {"role.imav.light.i2c_address", {.min = 0x44, .max = 0x47, .v = 0x44}},
      // The competition beacon should already be in its steady pattern. Enable
      // this only for bench tests which must also recognize its startup mode.
      {"role.imav.light.beginning_pattern", {.v = false}},
      // Nominal optical pulse timing. Both patterns share the high time but
      // have distinct low times. Read when the role starts; reboot after a
      // change. The detector derives frequency, duty cycle and harmonics.
      {"role.imav.light.high_ms", {.min = 50, .max = 200, .v = 100}},
      {"role.imav.light.steady_low_ms", {.min = 150, .max = 400, .v = 233}},
      {"role.imav.light.beginning_low_ms", {.min = 250, .max = 600, .v = 400}},
      // The VL53L4CX is optional and not fitted on the first IMAV assembly.
      // When disabled, no ToF initialization, retry or light pause occurs.
      {"role.imav.time_of_flight", {.v = false}},
      // Nominal VL53L4CX one-shot period. Small deterministic jitter is added
      // so a periodic beacon flash cannot always fall in the ToF light gap.
      {"role.imav.tof.period_ms", {.min = 100, .max = 1000, .v = 200}},
      // det is periodic; snr and lit follow recognized audio/light events.
      // Enable the remaining channels only while diagnosing the sensors.
      {"role.imav.debug.publish.optional", {.v = false}},
#endif

      // Optional linear correction on the measured battery voltage (psBat):
      // V_corrected = V_raw * adc.psbat.scale + adc.psbat.bias
      // ADC survey and shell calibration use these independently of the
      // optional voltmeter display role.
      {"adc.psbat.scale", {.min = 0.5f, .max = 1.5f, .v = 1.0f}},
      {"adc.psbat.bias", {.min = -5.0f, .max = 5.0f, .v = 0.0f}},

#if USE_VOLTMETER_ROLE
      {"ROLE.voltmeter", {.v = false}},
      {"role.voltmeter.cells", {.min = 2, .max = 6, .v = 4}},
      {"role.voltmeter.brightness", {.min = 0.0f, .max = 1.0f, .v = 0.2f}},

      // If >0 and a valid uavcan.equipment.gnss.Fix2 is received recently,
      // blank the WS2812 LEDs when ground speed is above this threshold.
      // Set to 0 to disable this behavior.
      {"role.voltmeter.gps_speed_off_mps", {.min = 0.0f, .max = 100.0f, .v = 3.0f}},
	
#endif
