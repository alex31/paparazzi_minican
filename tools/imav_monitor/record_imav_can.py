#!/usr/bin/env python3
"""Record every IMAV DroneCAN KeyValue message to a lossless CSV stream."""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path
import signal
import sys
import time
from collections import Counter
from datetime import datetime, timezone


DEFAULT_PYDRONECAN_DIR = Path("/home/alex/DEV/STM32/UAVCAN/pydronecan")


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Record all uavcan.protocol.debug.KeyValue messages from one "
            "IMAV node. A zero duration records until Ctrl-C."
        )
    )
    parser.add_argument("--interface", default="can0", help="SocketCAN interface")
    parser.add_argument("--source-node", type=int, default=10,
                        help="IMAV source node id (default: 10)")
    parser.add_argument("--bitrate", type=int, default=1_000_000,
                        help="CAN bitrate (default: 1000000)")
    parser.add_argument("--listener-node", type=int, default=120,
                        help="local DroneCAN listener node id (default: 120)")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="capture duration in seconds; 0 means unlimited")
    parser.add_argument("--output", type=Path,
                        help="output CSV; default: captures/imav_<UTC>.csv")
    parser.add_argument("--pydronecan-dir", type=Path,
                        default=DEFAULT_PYDRONECAN_DIR,
                        help="path to the local pydronecan checkout")
    arguments = parser.parse_args()
    if not 1 <= arguments.source_node <= 127:
        parser.error("--source-node must be in [1..127]")
    if not 1 <= arguments.listener_node <= 127:
        parser.error("--listener-node must be in [1..127]")
    if arguments.duration < 0:
        parser.error("--duration must be non-negative")
    return arguments


def default_output_path() -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return Path(__file__).resolve().parent / "captures" / f"imav_{stamp}.csv"


def main() -> int:
    arguments = parse_arguments()
    sys.path.insert(0, str(arguments.pydronecan_dir))
    try:
        import dronecan  # pylint: disable=import-outside-toplevel
    except ImportError as error:
        print(f"cannot import pydronecan from {arguments.pydronecan_dir}: {error}",
              file=sys.stderr)
        return 2

    output_path = (arguments.output or default_output_path()).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    stop_requested = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    node = dronecan.make_node(
        arguments.interface,
        node_id=arguments.listener_node,
        bitrate=arguments.bitrate,
    )
    started_monotonic = time.monotonic()
    started_wall_ns = time.time_ns()
    message_count = 0
    key_counts: Counter[str] = Counter()
    last_sync = started_monotonic

    with output_path.open("x", newline="", buffering=1) as output_file:
        writer = csv.writer(output_file)
        writer.writerow([
            "host_time_utc",
            "host_time_ns",
            "elapsed_s",
            "source_node_id",
            "transfer_id",
            "key",
            "value",
        ])

        def on_key_value(event: object) -> None:
            nonlocal message_count, last_sync
            transfer = event.transfer
            if transfer.source_node_id != arguments.source_node:
                return
            wall_ns = time.time_ns()
            elapsed = time.monotonic() - started_monotonic
            key = bytes(event.message.key).decode(
                "utf-8", errors="replace").rstrip("\0")
            value = float(event.message.value)
            writer.writerow([
                datetime.fromtimestamp(
                    wall_ns / 1_000_000_000, timezone.utc
                ).isoformat(timespec="microseconds"),
                wall_ns,
                f"{elapsed:.9f}",
                transfer.source_node_id,
                transfer.transfer_id,
                key,
                f"{value:.9g}",
            ])
            message_count += 1
            key_counts[key] += 1
            now = time.monotonic()
            if now - last_sync >= 1.0:
                output_file.flush()
                os.fsync(output_file.fileno())
                last_sync = now

        node.add_handler(dronecan.uavcan.protocol.debug.KeyValue, on_key_value)
        print(f"recording node {arguments.source_node} on {arguments.interface}")
        print(f"output: {output_path}")
        try:
            while not stop_requested:
                if (arguments.duration > 0 and
                        time.monotonic() - started_monotonic >= arguments.duration):
                    break
                node.spin(0.05)
        finally:
            output_file.flush()
            os.fsync(output_file.fileno())
            node.close()

    elapsed = time.monotonic() - started_monotonic
    print(f"recorded {message_count} messages in {elapsed:.3f} s")
    print("counts:", " ".join(
        f"{key}={count}" for key, count in sorted(key_counts.items())))
    print(f"saved: {output_path}")
    if message_count == 0:
        print("warning: no matching KeyValue message received", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
