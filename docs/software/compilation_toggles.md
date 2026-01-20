# Compilation Toggles

Compile-time role toggles are defined in
[COMMON/source/roleConf.h](../../COMMON/source/roleConf.h):
- USE_SERVO_ROLE
- USE_BARO_MPL3115A2_ROLE
- USE_QMC5883_ROLE
- USE_ESC_DSHOT_ROLE
- USE_RC_SBUS_ROLE
- USE_GPS_UBX_ROLE
- USE_SERIAL_STREAM_ROLE
- USE_LED2812_ROLE
- USE_VOLTMETER_ROLE
- USE_TEMPLATE_ROLE

These must be enabled for roles to be compiled in. Runtime enable is separate
and controlled by ROLE.* parameters.
