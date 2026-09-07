#!/usr/bin/env python3
"""Check accepted and rejected UBX configuration lifetimes with the ARM compiler."""
import argparse
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

PREAMBLE = """
#include <cstdint>
#include "gpsUbxDecoder.hpp"
bool pvt(const UBX::NavPvt&) { return true; }
bool dop(const UBX::NavDop&) { return true; }
bool sat(const UBX::NavSat&) { return true; }
inline constexpr UBX::DecoderConf global_config{pvt, dop, sat};
"""

ACCEPTED = {
    "global_config": "void test() { UBX::Decoder decoder(global_config); }",
    "local_static": """
        void test() {
            static const UBX::DecoderConf config{pvt, dop, sat};
            UBX::Decoder decoder(config);
        }
    """,
    "role_static_member": """
        struct Role {
            inline static constexpr UBX::DecoderConf config{pvt, dop, sat};
            UBX::Decoder decoder;
            Role() : decoder(config) {}
        };
        void test() { Role role; }
    """,
    "checked_forwarding": """
        UBX::Decoder make(UBX::StaticDecoderConf config) {
            return UBX::Decoder(config);
        }
        void test() { auto decoder = make(global_config); }
    """,
    "heap_decoder_static_config": """
        UBX::Decoder *test() { return new UBX::Decoder(global_config); }
    """,
    "copied_wrapper": """
        void test() {
            UBX::StaticDecoderConf checked(global_config);
            auto copy = checked;
            UBX::Decoder decoder(copy);
        }
    """,
}

REJECTED = {
    "automatic_config": """
        void test() {
            const UBX::DecoderConf config{pvt, dop, sat};
            UBX::Decoder decoder(config);
        }
    """,
    "automatic_constexpr_config": """
        void test() {
            constexpr UBX::DecoderConf config{pvt, dop, sat};
            UBX::Decoder decoder(config);
        }
    """,
    "temporary_config": """
        void test() { UBX::Decoder decoder(UBX::DecoderConf{pvt, dop, sat}); }
    """,
    "heap_config": """
        void test() {
            auto *config = new UBX::DecoderConf{pvt, dop, sat};
            UBX::Decoder decoder(*config);
        }
    """,
    "unchecked_forwarding": """
        UBX::Decoder make(const UBX::DecoderConf& config) {
            return UBX::Decoder(config);
        }
    """,
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("etl", type=Path, help="ETL include directory")
    parser.add_argument("--cxx", default="arm-none-eabi-g++")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    command = shlex.split(args.cxx) + [
        "-std=gnu++26", "-mcpu=cortex-m4", "-mthumb", "-fno-exceptions",
        "-fno-rtti", "-fno-threadsafe-statics", "-fsyntax-only",
        "-fdiagnostics-color=never", "-I" + str(args.etl.resolve()),
        "-I" + str(root / "COMMON/source"),
    ]
    failures = []
    environment = dict(os.environ, LC_ALL="C")
    with tempfile.TemporaryDirectory(prefix="ubx-lifetime-test-") as temp:
        for cases, should_compile in ((ACCEPTED, True), (REJECTED, False)):
            for name, body in cases.items():
                source = Path(temp) / (name + ".cpp")
                source.write_text(PREAMBLE + body)
                result = subprocess.run(command + [str(source)],
                                        capture_output=True, text=True, env=environment)
                actual = result.returncode == 0
                expected_diagnostic = actual or any(
                    token in result.stderr
                    for token in ("constant expression", "consteval", "immediate")
                )
                if actual != should_compile or not expected_diagnostic:
                    failures.append(name)
                    print(f"FAIL {name}\n{result.stdout}{result.stderr}")
                else:
                    print(f"OK {name}: {'accepted' if actual else 'rejected'}")
    if failures:
        raise SystemExit(f"Failed cases: {', '.join(failures)}")
    print(f"UBX lifetime: {len(ACCEPTED)} accepted and {len(REJECTED)} rejected cases OK")


if __name__ == "__main__":
    main()
