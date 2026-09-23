/**
 * @file roleConf.h
 * @brief Compile-time toggles for optional application roles.
 *
 * Set to false to completely drop a role from the build. Health survey stays
 * always compiled in to keep system diagnostics available.
 */
#define USE_SERVO_ROLE           true
#define USE_BARO_MPL3115A2_ROLE  true
#define USE_QMC5883_ROLE         true
#define USE_ESC_DSHOT_ROLE       true
#define USE_RC_SBUS_ROLE         true
#define USE_GPS_UBX_ROLE         true
#define USE_SERIAL_STREAM_ROLE   true
#define USE_LED2812_ROLE         true
#define USE_VOLTMETER_ROLE       true
#define USE_TEMPLATE_ROLE        false

// Sensor roles extracted from imav2026; runtime activation defaults to false.
// Numeric values also allow mcuconf.h and the C vendor driver to use them.
#ifndef USE_MICROPHONE_ROLE
#define USE_MICROPHONE_ROLE       1
#endif
#ifndef USE_OPT4060_ROLE
#define USE_OPT4060_ROLE          1
#endif
#ifndef USE_VL53L4CX_ROLE
#define USE_VL53L4CX_ROLE         1
#endif
