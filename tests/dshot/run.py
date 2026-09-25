#!/usr/bin/env python3
"""Exercise the shared DShot decoder on the host, with undefined-behavior checks."""
import argparse
from pathlib import Path
import subprocess
import tempfile


HARNESS = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "dshot_erps.h"
// Use the ARM compiler's fast-integer widths on the host as well.
#define uint_fast16_t uint32_t
#define uint_fast8_t uint32_t
#define DSHOT_DMA_DATA_LEN 16U
#define DSHOT_DMA_EXTRADATA_LEN 2U
static const uint32_t ERPS_BIT1_DUTY = 453U;
@DECODER@

static size_t encode(uint16_t packet, uint16_t *capture, unsigned start, int jitter)
{
  static const uint8_t gcr[16] = {25,27,18,19,29,21,22,23,26,9,10,11,30,13,14,15};
  uint32_t levels = 0;
  for (unsigned i = 0; i < 4; ++i)
    levels |= (uint32_t)gcr[(packet >> (4*i)) & 15] << (5*i);
  levels ^= levels >> 16; levels ^= levels >> 8; levels ^= levels >> 4;
  levels ^= levels >> 2; levels ^= levels >> 1;
  size_t n = 1;
  capture[0] = start;
  for (unsigned i = 1; i <= 21; ++i) {
    unsigned previous = (levels >> (21-i)) & 1;
    unsigned next = i == 21 ? 1 : ((levels >> (20-i)) & 1);
    if (previous != next) {
      int offset = jitter ? ((n & 1) ? 20 : -20) : 0;
      capture[n++] = (uint16_t)(start + i*ERPS_BIT1_DUTY + offset);
    }
  }
  return n;
}

int main(void)
{
  uint16_t capture[18];
  DshotErps ep;
  for (unsigned payload = 0; payload < 4096; ++payload) {
    uint16_t packet = (payload << 4) | (~(payload ^ (payload >> 4) ^ (payload >> 8)) & 15);
    for (unsigned variant = 0; variant < 4; ++variant) {
      // Poison unwritten entries: the DMA buffer is deliberately not cleared.
      memset(capture, 0xa5, sizeof(capture));
      size_t n = encode(packet, capture, (variant & 1) ? 65000 : 100, variant & 2);
      assert(n <= 18);
      DshotErpsSetFromFrame(&ep, processErpsDmaBuffer(capture, n));
      assert(DshotErpsCheckCrc4(&ep));
      assert(ep.ep.rawFrame == packet);
    }
    if (!DshotErpsIsEdt(&ep)) {
      uint32_t period = (payload & 511) << (payload >> 9);
      uint32_t expected = payload == 4095 ? 0 : (period ? 60000000U / period : UINT32_MAX);
      assert(DshotErpsGetRpm(&ep) == expected);
    }
    size_t n = encode(packet ^ 1U, capture, 100, 0);
    DshotErpsSetFromFrame(&ep, processErpsDmaBuffer(capture, n));
    assert(!DshotErpsCheckCrc4(&ep));
  }
  assert(processErpsDmaBuffer(NULL, 0) == 0);
  for (size_t n = 1; n < 8; ++n)
    assert(processErpsDmaBuffer(capture, n) == 0);
  assert(processErpsDmaBuffer(capture, 19) == 0);
  for (size_t i = 0; i < 18; ++i) capture[i] = 100;
  assert(processErpsDmaBuffer(capture, 18) == 0); // duplicate timestamps
  for (size_t i = 0; i < 18; ++i) capture[i] = i*4*ERPS_BIT1_DUTY;
  assert(processErpsDmaBuffer(capture, 18) == 0); // invalid run length
  for (size_t i = 0; i < 18; ++i) capture[i] = i*3*ERPS_BIT1_DUTY;
  assert(processErpsDmaBuffer(capture, 18) == 0); // exceeds 21 bits
  uint32_t random = 1;
  for (unsigned trial = 0; trial < 10000; ++trial) {
    for (size_t i = 0; i < 18; ++i) {
      random = random*1664525U + 1013904223U;
      capture[i] = random >> 16;
    }
    (void)processErpsDmaBuffer(capture, trial % 19);
  }
  puts("DShot: 4096 payloads x 4 capture variants, CRC errors, RPM edge cases and malformed captures OK");
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--various", type=Path,
                        default=Path.home() / "DEV/STM32/CHIBIOS/COMMON/various")
    parser.add_argument("--cc", default="cc")
    args = parser.parse_args()
    source = (args.various / "dshot_rpmCapture.c").read_text()
    signature = "static uint32_t processErpsDmaBuffer(const uint16_t *capture, size_t dmaLen)\n{"
    start = source.index(signature)
    decoder = source[start:source.index("\n}", start) + 2]
    with tempfile.TemporaryDirectory(prefix="dshot-test-") as directory:
        temp = Path(directory)
        (temp / "ch.h").write_text("#include <stdint.h>\n#include <stddef.h>\n#include <stdbool.h>\n")
        (temp / "hal.h").write_text("")
        (temp / "stdutil.h").write_text("")
        # Compile the real shared codec with only the unused RTOS includes stubbed.
        for name in ("dshot_erps.c", "dshot_erps.h"):
            (temp / name).write_bytes((args.various / name).read_bytes())
        (temp / "test.c").write_text(HARNESS.replace("@DECODER@", decoder))
        executable = temp / "test"
        subprocess.run([args.cc, "-std=gnu11", "-O1", "-g", "-fshort-enums",
                        "-fsanitize=undefined", "-fno-sanitize-recover=undefined",
                        "-I" + str(temp), str(temp / "test.c"),
                        str(temp / "dshot_erps.c"), "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
