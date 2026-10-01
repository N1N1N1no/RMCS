#!/usr/bin/env python3
"""Plot the four recorded yaw sweeps from ValueCollector CSV files."""

import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt


HERE = Path(__file__).resolve().parent
FILES = [
    ("First", HERE / "yaw_sweep_2026-10-01.csv"),
    ("Repeat 1", HERE / "yaw_repeat_1_2026-10-01.csv"),
    ("Repeat 2", HERE / "yaw_repeat_2_2026-10-01.csv"),
    ("Repeat 3", HERE / "yaw_repeat_3_2026-10-01.csv"),
]
ACTIVE = "/gimbal/yaw_sweep/active"
TARGET = "/gimbal/yaw_sweep/target_offset"
ANGLE = "/gimbal/yaw/angle"
ERROR = "/gimbal/yaw/control_angle_error"
SAMPLES_PER_SECOND = 200  # 300 CSV samples in each 1.5-second test phase.


def read_active_rows(path):
    with path.open(newline="") as stream:
        rows = [row for row in csv.DictReader(stream) if row[ACTIVE] == "1"]
    if not rows:
        raise ValueError(f"No active yaw sweep in {path}")
    return rows


def main():
    fig, (position_ax, error_ax) = plt.subplots(
        2, 1, figsize=(12, 7), sharex=True, layout="constrained"
    )
    for label, path in FILES:
        rows = read_active_rows(path)
        time = [i / SAMPLES_PER_SECOND for i in range(len(rows))]
        center = sum(float(row[ANGLE]) for row in rows[220:300]) / 80
        actual = [math.degrees(float(row[ANGLE]) - center) for row in rows]
        error = [math.degrees(float(row[ERROR])) for row in rows]
        position_ax.plot(time, actual, label=label, linewidth=1.3)
        error_ax.plot(time, error, label=label, linewidth=1.0)

    target_rows = read_active_rows(FILES[0][1])
    target_time = [i / SAMPLES_PER_SECOND for i in range(len(target_rows))]
    target = [math.degrees(float(row[TARGET])) for row in target_rows]
    position_ax.step(
        target_time, target, where="post", color="black", linestyle="--",
        linewidth=1.4, label="Commanded offset",
    )
    position_ax.set_ylabel("Yaw offset from start (deg)")
    position_ax.set_title("Yaw sweep: command and measured encoder angle")
    fig.legend(loc="outside upper center", ncol=5)
    position_ax.grid(True, alpha=0.3)

    error_ax.axhline(0, color="black", linewidth=0.8)
    error_ax.set_ylabel("Control angle error (deg)")
    error_ax.set_xlabel("Time since sweep start (s)")
    error_ax.grid(True, alpha=0.3)
    error_ax.set_xlim(0, 15)

    output = HERE / "yaw_four_runs.png"
    fig.savefig(output, dpi=160)
    print(output)


if __name__ == "__main__":
    main()
