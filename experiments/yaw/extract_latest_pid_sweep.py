#!/usr/bin/env python3
"""Extract the most recent completed yaw sweep from a ValueCollector CSV."""

import csv
import sys
from pathlib import Path


def main() -> None:
    if len(sys.argv) != 3:
        raise SystemExit("usage: extract_latest_pid_sweep.py LOGGER_CSV OUTPUT_CSV")
    source, output = map(Path, sys.argv[1:])
    latest = []
    current = []
    with source.open(newline="") as stream:
        reader = csv.DictReader(stream)
        columns = reader.fieldnames
        for row in reader:
            if row["/gimbal/yaw_sweep/active"] == "1":
                current.append(row)
            elif current:
                latest, current = current, []
    if current:
        raise SystemExit("latest sweep is still active; wait for completion")
    if not latest:
        raise SystemExit("no completed yaw sweep in logger")
    with output.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(latest)
    print(f"{output}: {len(latest)} samples, indices {latest[0]['index']}–{latest[-1]['index']}")


if __name__ == "__main__":
    main()
