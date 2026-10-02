#!/usr/bin/env python3
"""Compare C-car yaw PID sweeps using the same reference and sampling rate."""

import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


HERE = Path(__file__).resolve().parent
RUNS = [
    ("baseline Kθ=10 Kω=13", "pid_baseline_k10_k13.csv"),
    ("speed Kθ=10 Kω=6", "pid_k10_k6.csv"),
    ("speed Kθ=10 Kω=9", "pid_k10_k9.csv"),
    ("angle Kθ=12 Kω=9", "pid_k12_k9.csv"),
    ("angle Kθ=12 Kω=9 repeat", "pid_k12_k9_repeat.csv"),
    ("baseline Kθ=10 Kω=13 repeat", "pid_baseline_k10_k13_repeat.csv"),
    ("angle Kθ=12 Kω=11", "pid_k12_k11.csv"),
    ("angle Kθ=12 Kω=11 repeat", "pid_k12_k11_repeat.csv"),
]
DT = 0.005  # 1 kHz executor and ValueCollector write_interval=5


def load(filename):
    with (HERE / filename).open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 2500:
        raise ValueError(f"Incomplete sweep in {filename}: {len(rows)} samples")
    field = lambda name: np.array([float(row[name]) for row in rows])
    index = np.array([int(row["index"]) for row in rows])
    if np.any(np.diff(index) != 1):
        raise ValueError(f"Noncontinuous logger indices in {filename}")
    return {
        "time": (index - index[0]) * DT,
        "reference": field("/gimbal/yaw_sweep/target_offset"),
        "angle": np.unwrap(field("/gimbal/yaw/angle")),
        "error": field("/gimbal/yaw/control_angle_error"),
        "speed": field("/gimbal/yaw/velocity_imu"),
        "command_torque": field("/gimbal/yaw/control_torque"),
        "measured_torque": field("/gimbal/yaw/torque"),
        "remote_ready": field("/gimbal/yaw_sweep/remote_ready"),
        "active": field("/gimbal/yaw_sweep/active"),
    }


def metrics(run):
    ref, q, e = run["reference"], run["angle"], run["error"]
    changes = np.flatnonzero(np.diff(ref) != 0) + 1
    if len(changes) != 9 or not np.all(run["remote_ready"] == 1):
        raise ValueError("Sweep waveform or remote gate differs across tests")
    steps = []
    for step_number, start in enumerate(changes, start=1):
        stop = int(changes[step_number]) if step_number < len(changes) else len(ref)
        delta = ref[start] - ref[start - 1]
        sign = np.sign(delta)
        q_before = float(np.median(q[start - 10 : start]))
        progress = sign * (q[start:stop] - q_before)
        cross_10 = np.flatnonzero(progress >= 0.1 * abs(delta))
        cross_90 = np.flatnonzero(progress >= 0.9 * abs(delta))
        rise = float((cross_90[0] - cross_10[0]) * DT) if len(cross_10) and len(cross_90) else None
        arrival = float(cross_90[0] * DT) if len(cross_90) else None
        band = np.abs(e[start:stop]) <= 0.01
        qualified = np.convolve(band.astype(int), np.ones(20, dtype=int), mode="valid")
        stable = np.flatnonzero(qualified == 20)
        band_time = float(stable[0] * DT) if len(stable) else None
        steps.append({
            "number": step_number,
            "target_change_rad": float(delta),
            "rise_10_90_s": rise,
            "arrival_90_s": arrival,
            "time_to_0p01rad_band_s": band_time,
            "late_abs_error_deg": float(np.rad2deg(np.median(np.abs(e[stop - 50 : stop])))),
            "overshoot_deg": float(np.rad2deg(max(0.0, np.max(-sign * e[start:stop])))),
            "integrated_abs_error_rad_s": float(np.sum(np.abs(e[start:stop])) * DT),
        })
    reversals = steps[1:8]  # seven full ±0.2 rad transitions; omit half-sized edges
    median = lambda key: float(np.median([s[key] for s in reversals if s[key] is not None]))
    tau = run["measured_torque"]
    cmd = run["command_torque"]
    return {
        "samples": int(len(q)),
        "full_reversal_count": len(reversals),
        "median_rise_10_90_s": median("rise_10_90_s"),
        "median_arrival_90_s": median("arrival_90_s"),
        "median_time_to_0p01rad_band_s": median("time_to_0p01rad_band_s"),
        "median_late_abs_error_deg": median("late_abs_error_deg"),
        "median_overshoot_deg": median("overshoot_deg"),
        "mean_integrated_abs_error_rad_s": float(np.mean([s["integrated_abs_error_rad_s"] for s in reversals])),
        "measured_torque_p99_abs_Nm": float(np.quantile(np.abs(tau), 0.99)),
        "measured_torque_peak_abs_Nm": float(np.max(np.abs(tau))),
        "measured_torque_rms_Nm": float(np.sqrt(np.mean(tau**2))),
        "command_torque_p99_abs_Nm": float(np.quantile(np.abs(cmd), 0.99)),
        "command_torque_peak_abs_Nm": float(np.max(np.abs(cmd))),
        "yaw_speed_peak_abs_rad_s": float(np.max(np.abs(run["speed"]))),
        "steps": steps,
    }


def main():
    records = [(label, filename, load(filename)) for label, filename in RUNS]
    summary = {label: {"csv": filename, **metrics(run)} for label, filename, run in records}
    (HERE / "pid_comparison.json").write_text(json.dumps(summary, indent=2) + "\n")
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True, constrained_layout=True)
    for label, _, run in records:
        center = np.median(run["angle"][:200])
        axes[0].plot(run["time"], np.rad2deg(run["angle"] - center), label=label, linewidth=1)
        axes[1].plot(run["time"], np.rad2deg(run["error"]), label=label, linewidth=0.9)
        axes[2].plot(run["time"], run["measured_torque"], label=label, linewidth=0.9)
    reference = records[0][2]
    axes[0].step(reference["time"], np.rad2deg(reference["reference"]), where="post",
                 color="black", linestyle="--", label="target", linewidth=1.1)
    axes[0].set_ylabel("relative yaw (deg)")
    axes[1].set_ylabel("control angle error (deg)")
    axes[2].set_ylabel("measured torque (N m)")
    axes[2].set_xlabel("time since sweep start (s)")
    for ax in axes:
        ax.grid(alpha=0.3)
    axes[0].legend(ncol=2)
    fig.savefig(HERE / "pid_comparison.png", dpi=160)
    plt.close(fig)

    chosen = {
        "pid_baseline_k10_k13.csv": ("baseline 1", "#3465a4", "-"),
        "pid_baseline_k10_k13_repeat.csv": ("baseline 2", "#3465a4", "--"),
        "pid_k12_k11.csv": ("selected 1", "#e66100", "-"),
        "pid_k12_k11_repeat.csv": ("selected 2", "#e66100", "--"),
    }
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True, constrained_layout=True)
    for _, filename, run in records:
        if filename not in chosen:
            continue
        label, color, style = chosen[filename]
        center = np.median(run["angle"][:200])
        axes[0].plot(run["time"], np.rad2deg(run["angle"] - center),
                     color=color, linestyle=style, label=label, linewidth=1.2)
        axes[1].plot(run["time"], np.rad2deg(run["error"]),
                     color=color, linestyle=style, linewidth=1.0)
        axes[2].plot(run["time"], run["measured_torque"],
                     color=color, linestyle=style, linewidth=0.9)
    axes[0].step(reference["time"], np.rad2deg(reference["reference"]), where="post",
                 color="black", linestyle=":", label="target", linewidth=1.2)
    axes[0].set_ylabel("relative yaw (deg)")
    axes[1].set_ylabel("control angle error (deg)")
    axes[2].set_ylabel("motor-current torque (N m)")
    axes[2].set_xlabel("time since sweep start (s)")
    for ax in axes:
        ax.grid(alpha=0.3)
    fig.legend(loc="outside upper center", ncol=5)
    fig.savefig(HERE / "pid_selected_vs_baseline.png", dpi=160)
    plt.close(fig)
    for label, data in summary.items():
        print(f"{label}: arrival90={data['median_arrival_90_s']:.3f}s, "
              f"late error={data['median_late_abs_error_deg']:.3f}deg, "
              f"torque p99={data['measured_torque_p99_abs_Nm']:.2f}Nm, "
              f"IAE={data['mean_integrated_abs_error_rad_s']:.4f} rad*s")


if __name__ == "__main__":
    main()
