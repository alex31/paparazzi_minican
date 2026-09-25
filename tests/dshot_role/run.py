#!/usr/bin/env python3
"""Exercise the DShot role on the host, with undefined-behavior checks."""
import argparse
from pathlib import Path
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", default="c++")
    parser.add_argument("--cc", default="cc")
    parser.add_argument("--various", type=Path,
                        default=Path.home() / "DEV/STM32/CHIBIOS/COMMON/various")
    parser.add_argument("--uavcan", type=Path,
                        default=Path.home() / "DEV/STM32/UAVCAN")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="dshot-role-test-") as directory:
        temp = Path(directory)
        project = Path(__file__).resolve().parents[2]
        source = (args.various / "esc_dshot.c").read_text()
        header = (args.various / "esc_dshot.h").read_text()
        def function(signature):
            start = source.index(signature)
            return source[start:source.index("\n}", start) + 2]
        start = header.index("typedef struct {", header.index("telemetry packed"))
        types = header[start:header.index("typedef union {", start)]
        support = Path(__file__).with_name("telemetry_test_support.hpp").read_text()
        support = support.replace("@TYPES@", types)
        support = support.replace("@GETTER@", function("DshotTelemetry dshotGetTelemetry("))
        support = support.replace("@UPDATER@", function("static void updateTelemetryFromBidirEdt(const DshotErps *erps, DshotTelemetry *tlm)\n{"))
        (temp / "telemetry_test_support.hpp").write_text(support)
        (temp / "ch.h").write_text("#include <stdint.h>\n#include <stddef.h>\n")
        (temp / "hal.h").write_text("")
        (temp / "dshot_erps.h").write_bytes((args.various / "dshot_erps.h").read_bytes())
        role = (project / "COMMON/source/escDshotRole.cpp").read_text()
        start = role.index("void  EscDshot::periodic(void *)")
        periodic = role[start:role.index("\n}", start) + 2]
        harness = Path(__file__).with_name("role_telemetry_test.cpp").read_text()
        (temp / "role_test.cpp").write_text(harness.replace("@PERIODIC@", periodic))
        executable = temp / "role_test"
        for bidir, edt in ((1, 1), (1, 0), (0, 0)):
            subprocess.run([args.cxx, "-std=c++17", "-O1", "-g",
                            "-D_Static_assert=static_assert", "-fshort-enums",
                            "-I" + str(temp),
                            "-fsanitize=undefined", "-fno-sanitize-recover=undefined",
                            f"-DDSHOT_BIDIR={bidir}",
                            f"-DDSHOT_BIDIR_EXTENTED_TELEMETRY={edt}",
                            str(temp / "role_test.cpp"), "-o", str(executable)], check=True)
            subprocess.run([str(executable)], check=True)
        harness = Path(__file__).with_name("telemetry_metadata_test.cpp").read_text()
        crc = function("static inline uint8_t updateCrc8(uint8_t crc, uint8_t crc_seed)\n{") + "\n" + function("static uint8_t calculateCrc8(const uint8_t *Buf, const uint8_t BufLen)\n{")
        receiver = function("static noreturn void dshotTlmRec (void *arg)\n{")
        (temp / "metadata_test.cpp").write_text(harness.replace("@CRC@", crc).replace("@RECEIVER@", receiver))
        subprocess.run([args.cxx, "-std=c++17", "-O1", "-g",
                        "-D_Static_assert=static_assert", "-fshort-enums", "-I" + str(temp),
                        "-fsanitize=undefined", "-fno-sanitize-recover=undefined",
                        str(temp / "metadata_test.cpp"), "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True)
        subprocess.run([args.cc, "-std=gnu11", "-O1", "-g",
                        "-fsanitize=undefined", "-fno-sanitize-recover=undefined",
                        "-DCANARD_ENABLE_TAO_OPTION=1",
                        "-I" + str(args.uavcan / "DSDLC/include"),
                        "-I" + str(args.uavcan / "libcanard"),
                        str(Path(__file__).with_name("telemetry_wire_test.c")),
                        str(args.uavcan / "libcanard/canard.c"),
                        str(args.uavcan / "DSDLC/src/dronecan.protocol.FlexDebug.c"),
                        str(args.uavcan / "DSDLC/src/uavcan.equipment.esc.Status.c"),
                        "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
