# VL53L4CX component provenance

This directory contains only `Drivers/BSP/Components/vl53l4cx` from the
X-CUBE-TOF1 checkout referenced by MicroCAN's `imav2026` branch:

- Upstream: https://github.com/STMicroelectronics/x-cube-tof1
- Upstream release: **v3.4.3** (`c12709c`).
- Imported checkout: `94e172ede594842a9f95fa2cb16c352d13039b00`.
- License: [LICENSE.md](LICENSE.md), preserved with the original source headers.

The imported component includes one existing local change from
`20ea7bd7d17b186996c80ea04c57f1646bba3c33`: rename a shadowing local variable
from `dividend` to `rounded_hist_counts` in `VL53LX_events_per_spad_maths`.
No algorithm is changed. The subsequent deletion of example binaries does not
affect this component.

The local commits are not upstream revisions, so this component is vendored
instead of making a fresh clone depend on the local X-CUBE-TOF1 checkout.
Example projects, firmware binaries, compiled HTML documentation and web assets
are omitted. All C/H files and the license are preserved.

MicroCAN replaces `porting/vl53lx_platform.c` at build time with
`COMMON/source/vl53lxPlatform.c`. The replacement uses a scratch buffer owned
by the active role, checked transfer sizes, and sleeping RTOS delays. Its bus
callbacks combine register selection and reading under the shared I2C mutex.
MicroCAN also moves two upstream writable statics into `VL53L4CX_Object_t`:
the 92-byte multi-ranging result buffer and the 44-byte bare-driver tuning
table. The object is allocated only when the range role starts. The original
tuning defaults are retained as a `const` table in flash and copied on first
access for each zero-initialized device. This also isolates tuning changes
between instances. A compile-time size check follows the vendor tuning enum.
These changes are limited to `vl53l4cx.h`, `vl53l4cx.c` and
`modules/vl53lx_api.c`; the ranging algorithms and default values are unchanged.

All other component sources are compiled as imported. Warning exceptions
remain scoped to vendor objects, not to application code.
