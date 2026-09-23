#!/usr/bin/env python3
"""Regenerate the checked-in audio wire bindings using dronecan_dsdlc."""
import argparse
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", type=Path, default=
                        root.parents[3] / "UAVCAN/dronecan_dsdlc/dronecan_dsdlc.py")
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="microcan-audio-dsdl-") as temporary:
        temp = Path(temporary)
        subprocess.run([sys.executable, str(args.compiler), "-j1", "-O", str(temp),
                        str(root / "DSDL/microcan")], check=True)
        for folder, extension in (("include", "h"), ("src", "c")):
            for name in ("Bin", "Spectrum"):
                relative = Path(folder) / f"microcan.audio.{name}.{extension}"
                source = temp / relative
                target = root / "DSDL/generated" / relative
                if args.check:
                    if not target.exists() or source.read_bytes() != target.read_bytes():
                        raise SystemExit(f"Regenerate stale DSDL binding: {target}")
                else:
                    target.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copyfile(source, target)
    # Do not copy dronecan_msgs.h: it would shadow the shared standard bindings.


if __name__ == "__main__":
    main()
