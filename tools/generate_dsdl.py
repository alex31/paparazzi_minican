#!/usr/bin/env python3
"""Generate shared MicroCAN codecs from UAVCAN/DSDL on branch minican."""
import argparse
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


def main():
    root = Path(__file__).resolve().parents[1]
    shared = root.parents[3] / "UAVCAN"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", type=Path, default=
                        shared / "dronecan_dsdlc/dronecan_dsdlc.py")
    parser.add_argument("--dsdl-root", type=Path, default=shared / "DSDL")
    parser.add_argument("--output", type=Path, default=shared / "DSDLC")
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    branch = subprocess.check_output(
        ["git", "-C", str(args.dsdl_root), "branch", "--show-current"], text=True).strip()
    if branch != "minican":
        raise SystemExit(f"Custom DSDL must use branch minican, found {branch!r}")
    schemas = args.dsdl_root / "microcan"
    if not schemas.is_dir():
        raise SystemExit(f"Missing shared MicroCAN schemas: {schemas}")
    aggregate = args.output / "include/dronecan_msgs.h"
    if not aggregate.is_file():
        raise SystemExit(f"Generate standard shared bindings first: {aggregate}")
    names = []
    for schema in sorted(schemas.rglob("*.uavcan")):
        parts = schema.stem.split(".")
        name = parts[1] if parts[0].isdigit() else schema.stem
        namespace = schema.parent.relative_to(args.dsdl_root).parts
        names.append(".".join((*namespace, name)))
    if not names:
        raise SystemExit(f"No DSDL definitions in {schemas}")
    # Parse all shared namespaces to resolve dependencies and reject ID clashes.
    roots = [path for path in sorted(args.dsdl_root.iterdir())
             if path.is_dir() and not path.name.startswith(".") and path.name != "tests"]
    with tempfile.TemporaryDirectory(prefix="microcan-dsdl-") as temporary:
        temp = Path(temporary)
        subprocess.run([sys.executable, str(args.compiler), "-j1", "-O", str(temp),
                        *map(str, roots)], check=True)
        for folder, extension in (("include", "h"), ("src", "c")):
            for name in names:
                relative = Path(folder) / f"{name}.{extension}"
                source = temp / relative
                target = args.output / relative
                if args.check:
                    if not target.exists() or source.read_bytes() != target.read_bytes():
                        raise SystemExit(f"Regenerate stale DSDL binding: {target}")
                else:
                    target.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copyfile(source, target)
        # Refresh only MicroCAN includes; preserve other shared codecs/entries.
        prefix = '#include "microcan.'
        lines = [line for line in aggregate.read_text().splitlines()
                 if not line.startswith(prefix)]
        lines.extend(line for line in (temp / "include/dronecan_msgs.h").read_text().splitlines()
                     if line.startswith(prefix))
        updated = "\n".join(lines) + "\n"
        if args.check:
            if aggregate.read_text() != updated:
                raise SystemExit(f"Regenerate stale DSDL aggregate: {aggregate}")
        elif aggregate.read_text() != updated:
            aggregate.write_text(updated)


if __name__ == "__main__":
    main()
