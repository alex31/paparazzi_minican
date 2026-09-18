# Parameter System

Parameter list: [COMMON/source/nodeParameters.hpp](../../COMMON/source/nodeParameters.hpp).
Stored in external EEPROM via Persistant::Storage and exposed via UAVCAN GetSet.

Important parameters:

- uavcan.node_id (0 = dynamic ID)
- uavcan.dynid.fd (enable FD dynamic allocation)
- uavcan.param_set_behavior (SetRam / SetRamFlash / SetRamFlashAndReboot)
- bus.serial.baudrate (0..460800, where 0 enables GNSS UBX auto-baud probing)
- bus.i2c.frequency_khz
- ROLE.imav.beacon (enable the IMAV sound/light role)
- role.imav.audio.band_low_hz (2000..2600 Hz, default 2000; applied when the
  IMAV role starts, so reboot after changing it)
- role.imav.audio.snr_alpha (weight of the newest recognized burst in the SNR
  first-order IIR, 0.5..1.0, default 1.0; applied live, where 1.0 disables
  smoothing)
- role.imav.light.i2c_address (0x44..0x47, default 0x44 for OPT4060 ADDR=GND)
- role.imav.light.beginning_pattern (also recognize the roughly 2 Hz startup
  flash pattern; default false so the competition detector only accepts the
  steady pattern; reboot after changing it)
- role.imav.light.cree_test (bench tests with the CREE headlamp measured at
  7.99 Hz, approximately 50% duty; default false). When true, selects an 8 Hz
  profile accepting white or red-filtered flashes and overrides the startup
  pattern and MotionSCOUT pulse timings without changing their stored values.
  Save and reboot after changing it; set false and reboot for competition.
  See [measurement and test procedure](roles/cree_test.md).
- role.imav.light.adaptive_pattern (default false): add automatic recognition
  of the video's triplets/slow flashes alongside the original spectral path.
  Standard timing parameters and the startup toggle retain their effect.
  No timing adjustments are needed for the new motifs. Save and reboot; leave
  cree_test=false. See [adaptive light](roles/imav_adaptive_light.md).
- role.imav.light.high_ms (50..200 ms, default 100),
  role.imav.light.steady_low_ms (150..400 ms, default 233) and
  role.imav.light.beginning_low_ms (250..600 ms, default 400): nominal pulse
  timings used to derive the temporal, fundamental and harmonic filters;
  reboot after changing them
- role.imav.time_of_flight (enable the optional VL53L4CX, default false)
- role.imav.tof.period_ms (100..1000 ms, used only when time-of-flight is
  enabled; default 200 ms before deterministic jitter)
- role.imav.debug.publish.optional (publish the optional IMAV tuning keys in
  addition to event-driven `snr`/`lit`; derived tuning
  remains at 5 Hz while lossless RGBW samples follow OPT4060 data ready at up
  to about 139 Hz; default false, applied live)
- ROLE.* toggles for each role

Parameter handling is implemented in:
- [microcan/source/UAVCanSlave.cpp](../../microcan/source/UAVCanSlave.cpp) (GetSet, ExecuteOpcode)
- [COMMON/source/deviceResource.cpp](../../COMMON/source/deviceResource.cpp) (storage instance)
