#!/usr/bin/env python3
"""Host checks for the production sensor decoders and ST platform port."""
import os
import argparse
from pathlib import Path
import shlex
import subprocess
import tempfile


def main():
    root = Path(__file__).resolve().parents[2]
    common = root / "COMMON/source"
    tests = Path(__file__).resolve().parent
    vendor = root / "third_party/vl53l4cx"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--libcanard", type=Path,
                        default=root.parents[3] / "UAVCAN/libcanard")
    parser.add_argument("--dsdlc", type=Path,
                        default=root.parents[3] / "UAVCAN/DSDLC")
    args = parser.parse_args()
    cc = shlex.split(os.environ.get("HOST_CC", "cc"))
    cxx = shlex.split(os.environ.get("HOST_CXX", "c++"))
    flags = ["-Wall", "-Wextra", "-Werror", "-O2"]
    with tempfile.TemporaryDirectory(prefix="sensor-roles-") as directory:
        temp = Path(directory)
        decode = temp / "sensor_decode"
        subprocess.run(cxx + flags + ["-std=c++20", "-I" + str(common),
                       str(tests / "sensor_decode_test.cpp"), "-o", str(decode)], check=True)
        subprocess.run([str(decode)], check=True)
        spectrum = temp / "microphone_spectrum"
        subprocess.run(cxx + flags + ["-std=c++20", "-I" + str(common),
                       str(tests / "microphone_spectrum_test.cpp"),
                       "-o", str(spectrum)], check=True)
        subprocess.run([str(spectrum)], check=True)
        generated = args.dsdlc
        wire_flags = ["-DCANARD_ENABLE_TAO_OPTION=1", "-DCANARD_ENABLE_CANFD=1",
                      "-I" + str(common), "-I" + str(args.libcanard),
                      "-I" + str(generated / "include")]
        wire_objects = []
        codecs = [generated / "src" / f"{name}.c" for name in
                  ("microcan.audio.Bin", "microcan.audio.Spectrum", "microcan.light.Measurement")]
        for source in [args.libcanard / "canard.c", *codecs]:
            obj = temp / (source.stem + ".o")
            subprocess.run(cc + flags + ["-std=c11"] + wire_flags +
                           ["-c", str(source), "-o", str(obj)], check=True)
            wire_objects.append(str(obj))
        wire = temp / "audio_message"
        subprocess.run(cxx + flags + ["-std=c++20"] + wire_flags +
                       [str(tests / "audio_message_test.cpp")] + wire_objects +
                       ["-o", str(wire)], check=True)
        subprocess.run([str(wire)], check=True)
        light = temp / "light"
        subprocess.run(cxx + flags + ["-std=c++23"] + wire_flags +
                       [str(tests / "light_test.cpp")] + wire_objects +
                       ["-o", str(light)], check=True)
        subprocess.run([str(light)], check=True)
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
