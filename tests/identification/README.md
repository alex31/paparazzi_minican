# Identification startup and LED regression

From the project root:

```sh
python3 tests/identification/run.py
python3 tests/identification/run_led.py
```

The harness executes the production `main()` and `CANSlave` startup functions
with hardware, role and UAVCAN transport doubles. It uses the shared generated
DroneCAN headers in `~/DEV/STM32/UAVCAN/DSDLC`; it does not copy DSDL schemas.

It verifies, with and without `TRACE`:

- Identification reaches node startup and registers parameter, restart, node-info,
  firmware-update and file-read handlers, even with failing configured roles.
- Application roles are not constructed/subscribed/started, except the shell
  when enabled. The maintenance status and purple motif survive startup,
  including simulated dynamic node-ID allocation.
- Identification starts the shell only when ROLE.shell is true and TRACE is
  compiled in. Shell startup failure leaves management services reachable.
- Changing the parameter while node startup is in progress does not change the
  selected boot mode or replace the identification LED with the node-ID display.
- After normal startup the installed LED reader observes changes immediately,
  without changing role counts, health survey or NodeStatus. Enabling the
  parameter during node startup does not change the boot role selection either.
- Normal startup still starts the roles and health survey, while a role resource
  error still selects the error LED. Shell on/off is checked separately, and
  builds without TRACE never instantiate the shell.

The LED suite compiles the entire production `rgbLeds.cpp` with a fake clock
and LED transport, using the real constexpr color converter from the shared
`CHIBIOS/COMMON/various/led2812.hpp`. It checks base-5 digit colors and timing,
time-counter wrap, transport delay, activation during every digit and pause,
the purple 150 ms motif, return to the first ID digit within 200 ms, and
fault-animation priority. It also checks the minimal bootloader and static
motifs, including constant colors and all-off patterns.

Every animation sleep must be positive and at most 200 ms. Repeated identical
frames are rejected. The 3.5 s node-ID scenario takes 21 sleeps and 7 frames;
a constant motif takes 18 sleeps and one frame. The fault rainbow retains its
10 ms step. These counts are simulation results, not CPU measurements on hardware.

The transport double checks handler reachability, not actual CAN communication,
dynamic allocation timing, flash writes or callback internals. Validate parameter
changes, restart and a complete firmware update on hardware with test T16 in
[`procedure_de_test.md`](../../procedure_de_test.md). Firmware builds additionally
check the real HAL/DroneCAN integration.
