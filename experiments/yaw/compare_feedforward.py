#!/usr/bin/env python3
"""Compare identical smooth yaw sweeps with and without model feedforward."""

import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parent
DT = 0.005
TRANSITION = 0.5
HOLD = 1.5
FILES = {
    "off": ROOT / "ff_off_smooth_2026-10-02.csv",
    "velocity_only": ROOT / "ff_velocity_only_smooth_2026-10-02.csv",
    "on": ROOT / "ff_on_smooth_2026-10-02.csv",
}
GAINS = {
    "off": {"reference_velocity": 0.0, "model_velocity": 0.0, "model_acceleration": 0.0},
    "velocity_only": {"reference_velocity": 1.0, "model_velocity": 0.0, "model_acceleration": 0.0},
    "on": {"reference_velocity": 1.0, "model_velocity": 1.378547846, "model_acceleration": 0.142611646},
}


def load(path):
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 2800:
        raise ValueError(f"Incomplete sweep: {path} has {len(rows)} samples")
    index = np.array([int(row["index"]) for row in rows])
    if np.any(np.diff(index) != 1):
        raise ValueError(f"Noncontinuous indices in {path}")
    def field(name):
        return np.array([float(row[name]) for row in rows])
    data = {
        "t": (index - index[0]) * DT,
        "reference": field("/gimbal/yaw_sweep/target_offset"),
        "reference_velocity": field("/gimbal/yaw_sweep/reference_velocity"),
        "reference_acceleration": field("/gimbal/yaw_sweep/reference_acceleration"),
        "scale": field("/gimbal/yaw_sweep/feedforward_scale"),
        "angle": np.unwrap(field("/gimbal/yaw/angle")),
        "error": field("/gimbal/yaw/control_angle_error"),
        "speed": field("/gimbal/yaw/velocity_imu"),
        "command_torque": field("/gimbal/yaw/control_torque"),
        "measured_torque": field("/gimbal/yaw/torque"),
        "remote_ready": field("/gimbal/yaw_sweep/remote_ready"),
        "active": field("/gimbal/yaw_sweep/active"),
    }
    if not np.all(data["remote_ready"] == 1) or not np.all(data["active"] == 1):
        raise ValueError(f"Remote gate or sweep inactive in {path}")
    return data


def metrics(data):
    t = data["t"]
    e = data["error"]
    moving = np.zeros(len(t), dtype=bool)
    reversals = []
    for phase in range(2, 9):
        start = phase * HOLD
        segment = (t >= start) & (t < start + HOLD)
        motion = (t >= start) & (t < start + TRANSITION)
        moving |= motion
        if not np.any(motion) or not np.any(segment):
            raise ValueError("Missing full reversal phase")
        reversals.append({
            "phase": phase,
            "moving_iae_rad_s": float(np.sum(np.abs(e[motion])) * DT),
            "moving_peak_abs_error_deg": float(np.rad2deg(np.max(np.abs(e[motion])))),
            "late_abs_error_deg": float(np.rad2deg(np.median(np.abs(e[segment][-50:])))),
        })
    torque = data["measured_torque"]
    command = data["command_torque"]
    return {
        "samples": len(t),
        "feedforward_scale": float(np.median(data["scale"])),
        "reversal_count": len(reversals),
        "mean_moving_iae_rad_s": float(np.mean([s["moving_iae_rad_s"] for s in reversals])),
        "moving_rms_error_deg": float(np.rad2deg(np.sqrt(np.mean(e[moving] ** 2)))),
        "median_late_abs_error_deg": float(np.median([s["late_abs_error_deg"] for s in reversals])),
        "speed_peak_abs_rad_s": float(np.max(np.abs(data["speed"]))),
        "measured_torque_p99_abs_Nm": float(np.quantile(np.abs(torque), 0.99)),
        "measured_torque_peak_abs_Nm": float(np.max(np.abs(torque))),
        "command_torque_peak_abs_Nm": float(np.max(np.abs(command))),
        "command_limit_fraction": float(np.mean(np.abs(command) > 4.0)),
        "reversals": reversals,
    }


def main():
    runs = {name: load(path) for name, path in FILES.items()}
    for name in runs:
        if abs(len(runs[name]["t"]) * DT - 15.0) > 0.1:
            raise ValueError(f"Unexpected sweep duration in {name}")
    common_time = np.arange(0.0, 14.99, DT)
    off_reference = np.interp(common_time, runs["off"]["t"], runs["off"]["reference"])
    for name in ("velocity_only", "on"):
        reference = np.interp(common_time, runs[name]["t"], runs[name]["reference"])
        if np.max(np.abs(off_reference - reference)) > 0.005:
            raise ValueError(f"Reference waveform differs in {name}")
    if abs(np.median(runs["off"]["scale"])) > 0.01 or any(
        abs(np.median(runs[name]["scale"]) - 1.0) > 0.01 for name in ("velocity_only", "on")
    ):
        raise ValueError("Unexpected feedforward scale")
    result = {
        name: {"csv": str(FILES[name]), "effective_gains": GAINS[name], **metrics(data)}
        for name, data in runs.items()
    }
    (ROOT / "feedforward_comparison.json").write_text(json.dumps(result, indent=2) + "\n")

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True, constrained_layout=True)
    colors = {"off": "#3465a4", "velocity_only": "#6e9b3d", "on": "#e66100"}
    for name, data in runs.items():
        center = np.median(data["angle"][:200])
        axes[0].plot(data["t"], np.rad2deg(data["angle"] - center), label=f"FF {name}", color=colors[name], linewidth=1)
        axes[1].plot(data["t"], np.rad2deg(data["error"]), color=colors[name], linewidth=0.9)
        axes[2].plot(data["t"], data["measured_torque"], color=colors[name], linewidth=0.8)
    off = runs["off"]
    axes[0].plot(off["t"], np.rad2deg(off["reference"]), "k--", linewidth=1, label="target")
    axes[0].set_ylabel("relative yaw (deg)")
    axes[1].set_ylabel("angle error (deg)")
    axes[2].set_ylabel("measured torque (N m)")
    axes[2].set_xlabel("time (s)")
    for ax in axes:
        ax.grid(alpha=0.3)
    axes[0].legend()
    fig.savefig(ROOT / "feedforward_comparison.png", dpi=160)
    plt.close(fig)
    for name, record in result.items():
        print(name, {k: record[k] for k in ("mean_moving_iae_rad_s", "moving_rms_error_deg", "measured_torque_p99_abs_Nm", "speed_peak_abs_rad_s")})


if __name__ == "__main__":
    main()
