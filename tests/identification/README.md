# Identification startup regression

Run `python3 tests/identification/run.py` from the project root.

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
- Normal startup still starts the roles and health survey, while a role resource
  error still selects the error LED. Shell on/off is checked separately, and
  builds without TRACE never instantiate the shell.

The transport double checks handler reachability, not actual CAN communication,
dynamic allocation timing, flash writes or callback internals. Validate parameter
changes, restart and a complete firmware update on hardware with test T16 in
[`procedure_de_test.md`](../../procedure_de_test.md). Firmware builds additionally
check the real HAL/DroneCAN integration.
