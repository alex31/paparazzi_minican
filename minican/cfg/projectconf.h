// local defs
#define CH_HEAP_SIZE (28*1024)
#define DMA_HEAP_SIZE (4*1024)
#define CH_HEAP_USE_TLSF 0 // if 0 or undef, chAlloc will be used
#define CONSOLE_DEV_SD LPSD1
#define CONSOLE_DEV_USB 0
#define CHPRINTF_USE_STDLIB   1



// Compile-time toggles for optional application roles.
// Set to false to completely drop a role from the build.
// Health survey stays always compiled in to keep system diagnostics available.
#define USE_SERVO_ROLE           true
#define USE_BARO_MPL3115A2_ROLE  true
#define USE_QMC5883_ROLE         true
#define USE_ESC_DSHOT_ROLE       true
#define USE_RC_SBUS_ROLE         true
#define USE_GPS_UBX_ROLE         true
#define USE_PPRZLINK_TUNNEL_ROLE true


