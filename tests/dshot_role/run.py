#!/usr/bin/env python3
"""Exercise the DShot role on the host, with undefined-behavior checks."""
import argparse
from pathlib import Path
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", default="c++")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="dshot-role-test-") as directory:
        temp = Path(directory)
        project = Path(__file__).resolve().parents[2]
        role = (project / "COMMON/source/escDshotRole.cpp").read_text()
        start = role.index("void  EscDshot::periodic(void *)")
        periodic = role[start:role.index("\n}", start) + 2]
        harness = Path(__file__).with_name("role_telemetry_test.cpp").read_text()
        (temp / "role_test.cpp").write_text(harness.replace("@PERIODIC@", periodic))
        executable = temp / "role_test"
        for bidir, edt in ((1, 1), (1, 0), (0, 0)):
            subprocess.run([args.cxx, "-std=c++17", "-O1", "-g",
                            "-fsanitize=undefined", "-fno-sanitize-recover=undefined",
                            f"-DDSHOT_BIDIR={bidir}",
                            f"-DDSHOT_BIDIR_EXTENTED_TELEMETRY={edt}",
                            str(temp / "role_test.cpp"), "-o", str(executable)], check=True)
            subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
