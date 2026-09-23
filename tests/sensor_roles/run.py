#!/usr/bin/env python3
"""Host checks for the production sensor decoders and ST platform port."""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile


def main():
    root = Path(__file__).resolve().parents[2]
    common = root / "COMMON/source"
    tests = Path(__file__).resolve().parent
    vendor = root / "third_party/vl53l4cx"
    cc = shlex.split(os.environ.get("HOST_CC", "cc"))
    cxx = shlex.split(os.environ.get("HOST_CXX", "c++"))
    flags = ["-Wall", "-Wextra", "-Werror", "-O2"]
    with tempfile.TemporaryDirectory(prefix="sensor-roles-") as directory:
        temp = Path(directory)
        decode = temp / "sensor_decode"
        subprocess.run(cxx + flags + ["-std=c++20", "-I" + str(common),
                       str(tests / "sensor_decode_test.cpp"), "-o", str(decode)], check=True)
        subprocess.run([str(decode)], check=True)
        includes = ["-I" + str(path) for path in
                    (common, vendor, vendor / "modules", vendor / "porting")]
        port = temp / "port.o"
        subprocess.run(cc + flags + ["-std=c11"] + includes +
                       ["-c", str(common / "vl53lxPlatform.c"), "-o", str(port)], check=True)
        platform = temp / "vl53lx_platform"
        subprocess.run(cxx + flags + ["-std=c++20"] + includes +
                       [str(tests / "vl53lx_platform_test.cpp"), str(port),
                        "-o", str(platform)], check=True)
        subprocess.run([str(platform)], check=True)
        api = temp / "api.o"
        # Discard unrelated vendor API entry points and hardware dependencies.
        subprocess.run(cc + ["-std=c11", "-O2", "-ffunction-sections", "-fdata-sections"] +
                       includes + ["-c", str(vendor / "modules/vl53lx_api.c"),
                                   "-o", str(api)], check=True)
        tuning = temp / "vl53lx_tuning"
        subprocess.run(cxx + flags + ["-std=c++20", "-Wl,--gc-sections"] + includes +
                       [str(tests / "vl53lx_tuning_test.cpp"), str(api),
                        "-o", str(tuning)], check=True)
        subprocess.run([str(tuning)], check=True)


if __name__ == "__main__":
    main()
