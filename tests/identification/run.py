#!/usr/bin/env python3
"""Exercise the production boot paths with host doubles for hardware and UAVCAN."""
import argparse
from pathlib import Path
import subprocess
import tempfile


def function(source, signature):
    start = source.index(signature)
    # Both production functions close at column zero; nested blocks are indented.
    return source[start:source.index("\n}", start) + 2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", default="c++")
    parser.add_argument("--uavcan", type=Path,
                        default=Path.home() / "DEV/STM32/UAVCAN")
    args = parser.parse_args()
    project = Path(__file__).resolve().parents[2]
    startup = (project / "microcan/source/UAVCanSlave.cpp").read_text()
    # Include the real namespace (getNodeId/getInstance/start), not a copy of its
    # control flow. Its management callbacks are recorded by the host Node.
    startup = startup[startup.index("namespace CANSlave {"):]
    startup = startup[:startup.index("\n}\n") + 2]
    entry = function((project / "microcan/source/main.cpp").read_text(),
                     "int main(void)").replace("int main(void)", "int firmwareMain()", 1)
    bits = (project / "COMMON/source/roleStatus.hpp").read_text()
    bits = bits[bits.index("enum SpecificCodeBits"):]
    harness = Path(__file__).with_name("startup_test.cpp").read_text()
    harness = harness.replace("@STARTUP@", startup).replace("@MAIN@", entry)
    harness = harness.replace("@STATUS_BITS@", bits)
    with tempfile.TemporaryDirectory(prefix="identification-test-") as directory:
        source = Path(directory) / "startup_test.cpp"
        source.write_text(harness)
        binary = Path(directory) / "startup_test"
        for trace in (False, True):
            subprocess.run([args.cxx, "-std=c++20", "-O0", "-g", "-Wall", "-Wextra",
                            "-Werror", "-Wno-missing-field-initializers",
                            "-fsanitize=undefined", "-fno-sanitize-recover=undefined",
                            "-I" + str(args.uavcan / "libcanard"),
                            "-I" + str(args.uavcan / "DSDLC/include"),
                            *(["-DTRACE"] if trace else []),
                            str(source), "-o", str(binary)], check=True)
            for scenario in ("identification", "identification-shell-off",
                             "identification-shell-error", "dynamic", "changed-during-start",
                             "normal", "shell-off", "role-error"):
                # Each invocation models a reboot, including production statics.
                subprocess.run([str(binary), scenario], check=True)


if __name__ == "__main__":
    main()
