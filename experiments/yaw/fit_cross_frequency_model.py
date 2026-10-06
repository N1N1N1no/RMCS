#!/usr/bin/env python3
"""Fit and validate a second-order yaw model with static/kinetic friction.

The train/held-out split is by complete robot runs. The low-amplitude chirps
that barely move are included in the holdout assessment rather than discarded.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import fit_yaw_plant as plant
from analyze_large_chirp_validation import load_chirp


ROOT = Path(__file__).resolve().parent
OUT = ROOT / "cross_frequency_model_2026-10-06"
FILES = {
    "low004_a": ROOT / "cross_frequency_2026-10-06/01_low_004.csv",
    "mid004_a": ROOT / "cross_frequency_2026-10-06/02_mid_004.csv",
    "high004_a": ROOT / "cross_frequency_2026-10-06/03_high_004.csv",
    "low004_b": ROOT / "cross_frequency_2026-10-06/04_low_004_repeat.csv",
    "mid004_b": ROOT / "cross_frequency_2026-10-06/05_mid_004_repeat.csv",
    "high004_b": ROOT / "cross_frequency_followup_2026-10-06/01_high_004_repeat.csv",
    "low008": ROOT / "cross_frequency_followup_2026-10-06/02_low_008.csv",
    "low011": ROOT / "cross_frequency_followup_2026-10-06/03_low_011.csv",
    "mid008": ROOT / "cross_frequency_followup_2026-10-06/04_mid_008.csv",
}
TRAIN = ["low004_a", "mid004_a", "high004_a", "low008"]
HOLDOUT = ["low004_b", "mid004_b", "high004_b", "low011", "mid008"]
WINDOWS = [(1.0, 3.0), (3.0, 5.0), (5.0, 7.0)]
REFERENCE_ANGLE = 4.55
OLD = json.loads((ROOT / "yaw_plant_fit.json").read_text())
J_OLD = OLD["parameters"]["J_kg_m2"]
B_OLD = OLD["parameters"]["B_Nm_s_per_rad"]
DELAY_SAMPLES = OLD["delay_ms"]


def downsample(path: Path) -> dict[str, np.ndarray]:
    run = load_chirp(path)
    # 200 Hz suffices for the 0.3-3 Hz trajectories and makes deterministic
    # parameter search cheap. Delay is applied before decimation.
    take = np.arange(5, len(run["time"]) - 1, 5)
    return {key: value[take] for key, value in run.items()} | {
        "delayed_torque": run["torque"][take - DELAY_SAMPLES]
    }


def simulate(run: dict[str, np.ndarray], start: float, end: float,
             params: dict[str, float]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    t = run["time"]
    lo = int(np.searchsorted(t, start))
    hi = int(np.searchsorted(t, end))
    q = float(run["angle"][lo])
    # Average the initial IMU rate to reduce start-state noise at low speed.
    v = float(np.mean(run["speed"][max(0, lo - 3):lo + 4]))
    angles = np.empty(hi - lo)
    speeds = np.empty(hi - lo)
    angles[0] = q
    speeds[0] = v
    J, B = params["J"], params["B"]
    Fc, Fs = params.get("Fc", 0.0), params.get("Fs", 0.0)
    K, d = params.get("K", 0.0), params["d"]
    stick_speed = 0.006
    for j, i in enumerate(range(lo + 1, hi), start=1):
        dt = t[i] - t[i - 1]
        net = run["delayed_torque"][i - 1] - d - K * (q - REFERENCE_ANGLE)
        if "v0" in params:
            # Implicit step keeps the smooth friction law stable at v0 << dt.
            v_next = v
            v0 = params["v0"]
            for _ in range(5):
                th = np.tanh(v_next / v0)
                function = v_next - v - dt * (net - B * v_next - Fc * th) / J
                slope = 1.0 + dt * (B + Fc * (1.0 - th * th) / v0) / J
                v_next -= function / slope
        elif abs(v) < stick_speed and abs(net) <= Fs:
            v_next = 0.0
        else:
            direction = np.sign(v) if abs(v) >= stick_speed else np.sign(net)
            v_next = v + dt * (net - B * v - Fc * direction) / J
            if v * v_next < 0.0 and abs(net) <= Fs:
                v_next = 0.0
        q += dt * v_next
        v = v_next
        angles[j] = q
        speeds[j] = v
    return t[lo:hi], angles, speeds


def old_disturbance(path: Path) -> float:
    run = load_chirp(path)
    obs = plant.observations(run, DELAY_SAMPLES)
    early = obs["time"] < 1.0
    return float(np.mean(obs["torque"][early] - B_OLD * obs["speed"][early]
                         - J_OLD * obs["acceleration"][early]))


def window_metrics(run: dict[str, np.ndarray], params: dict[str, float]) -> dict:
    angles = []
    speeds = []
    traces = []
    for start, end in WINDOWS:
        t, qhat, vhat = simulate(run, start, end, params)
        lo = int(np.searchsorted(run["time"], start))
        hi = lo + len(t)
        q, v = run["angle"][lo:hi], run["speed"][lo:hi]
        angles.append(float(np.sqrt(np.mean((qhat - q)**2))))
        speeds.append(float(np.sqrt(np.mean((vhat - v)**2))))
        traces.append((t, q, qhat))
    return {"angle_rmse_rad": float(np.mean(angles)),
            "angle_rmse_deg": float(np.rad2deg(np.mean(angles))),
            "velocity_rmse_rad_s": float(np.mean(speeds)),
            "angle_rmse_by_window_deg": np.rad2deg(angles).tolist(),
            "traces": traces}


def objective(runs: dict[str, dict], params: dict[str, float]) -> float:
    losses = []
    for name in TRAIN:
        run = runs[name]
        metric = window_metrics(run, params)
        angle_scale = max(float(np.std(run["angle"])), 0.012)
        speed_scale = max(float(np.std(run["speed"])), 0.06)
        losses.append(metric["angle_rmse_rad"] / angle_scale
                      + 0.25 * metric["velocity_rmse_rad_s"] / speed_scale)
    return float(np.mean(losses))


def fit(runs: dict[str, dict]) -> tuple[dict[str, float], list[dict]]:
    # Keep the independently measured inertia fixed; high-frequency data also
    # identified approximately this value. Search only physically plausible
    # positive friction and damping. Static friction cannot be below kinetic.
    params = {"J": J_OLD, "B": 0.5, "Fc": 0.4, "Fs": 0.62,
              "K": 0.5, "d": 0.0}
    ranges = {"B": (0.15, 1.4), "Fc": (0.1, 0.8), "Fs": (0.35, 1.2),
              "K": (-1.5, 2.5), "d": (-0.35, 0.35)}
    steps = {"B": 0.35, "Fc": 0.2, "Fs": 0.2, "K": 0.6, "d": 0.12}
    trace = []
    for round_index in range(5):
        for key in ("Fs", "Fc", "B", "d", "K"):
            current = params[key]
            candidates = np.linspace(current - steps[key], current + steps[key], 7)
            best = (float("inf"), current)
            for value in candidates:
                value = float(np.clip(value, *ranges[key]))
                candidate = params | {key: value}
                if candidate["Fs"] < candidate["Fc"]:
                    continue
                loss = objective(runs, candidate)
                if loss < best[0]:
                    best = (loss, value)
            params[key] = best[1]
        trace.append({"round": round_index + 1, "loss": objective(runs, params),
                      "parameters": params.copy()})
        for key in steps:
            steps[key] *= 0.5
    return params, trace


def fit_linear(runs: dict[str, dict]) -> dict[str, float]:
    """Fair linear comparator fitted on the identical training runs/loss."""
    params = {"J": J_OLD, "B": B_OLD, "K": 0.0, "d": 0.0}
    steps = {"B": 0.8, "K": 1.0, "d": 0.3}
    for _ in range(6):
        for key in ("d", "B", "K"):
            best = (float("inf"), params[key])
            for value in np.linspace(params[key] - steps[key],
                                     params[key] + steps[key], 9):
                if key == "B" and value <= 0:
                    continue
                candidate = params | {key: float(value)}
                loss = objective(runs, candidate)
                if loss < best[0]:
                    best = (loss, float(value))
            params[key] = best[1]
        for key in steps:
            steps[key] *= 0.5
    return params


def training_run_sensitivity(runs: dict[str, dict]) -> dict[str, dict[str, float]]:
    """Show parameter variation when one independent training run is omitted."""
    selected = TRAIN.copy()
    results = {}
    try:
        for omitted in selected:
            TRAIN[:] = [name for name in selected if name != omitted]
            results[omitted] = fit(runs)[0]
    finally:
        TRAIN[:] = selected
    return results


def sensor_consistency(path: Path) -> float:
    run = load_chirp(path)
    t, q, v = run["time"], run["angle"], run["speed"]
    errors = []
    for start, end in WINDOWS:
        lo, hi = np.searchsorted(t, [start, end])
        tt, qq, vv = t[lo:hi], q[lo:hi], v[lo:hi]
        integrated = qq[0] + np.cumsum(np.r_[0.0, np.diff(tt) *
                                               (vv[1:] + vv[:-1]) / 2.0])
        errors.append(np.rad2deg(np.sqrt(np.mean((integrated - qq)**2))))
    return float(np.mean(errors))


def plot_excitation_evidence() -> None:
    selected = [("low004_a", "0.04 rad, low"),
                ("low008", "0.08 rad, low"),
                ("low011", "0.11 rad, low")]
    fig, axes = plt.subplots(2, 3, figsize=(14, 7), constrained_layout=True)
    for column, (name, label) in enumerate(selected):
        raw = np.genfromtxt(FILES[name], delimiter=",", names=True)
        t = raw["t_s"]
        q = np.unwrap(raw["angle_rad"])
        target = raw["target_offset_rad"]
        speed = raw["gimbal_imu_rad_s"] - raw["chassis_imu_rad_s"]
        torque = raw["torque_measured_nm"]
        axes[0, column].plot(t, np.rad2deg(target), "k--", lw=1,
                             label="target")
        axes[0, column].plot(t, np.rad2deg(q - q[0]), color="tab:blue", lw=1,
                             label="encoder")
        axes[0, column].set(title=label, ylim=(-7, 7), xlabel="time (s)",
                             ylabel="yaw from start (deg)")
        keep = (t > 1.0) & (t < 7.9)
        indices = np.flatnonzero(keep)[::20]
        axes[1, column].scatter(speed[indices], torque[indices],
                                c=t[indices], cmap="viridis", s=7, alpha=0.7)
        axes[1, column].set(xlim=(-0.45, 0.45), ylim=(-0.95, 0.95),
                             xlabel="relative yaw speed (rad/s)",
                             ylabel="measured torque (N m)")
        for ax in axes[:, column]:
            ax.grid(alpha=0.25)
    axes[0, 0].legend()
    fig.savefig(OUT / "low_frequency_amplitude_evidence.png", dpi=160)
    plt.close(fig)


def main() -> None:
    OUT.mkdir(exist_ok=True)
    runs = {name: downsample(path) for name, path in FILES.items()}
    params, search_trace = fit(runs)
    linear = fit_linear(runs)
    sensitivity = training_run_sensitivity(runs)
    rows = []
    fig, axes = plt.subplots(3, 3, figsize=(15, 10), constrained_layout=True)
    for ax, (name, run) in zip(axes.flat, runs.items()):
        learned = window_metrics(run, params)
        fitted_linear = window_metrics(run, linear)
        baseline = window_metrics(run, {"J": J_OLD, "B": B_OLD,
                                        "d": old_disturbance(FILES[name])})
        row = {"run": name, "partition": "train" if name in TRAIN else "held_out",
               "initial_angle_rad": float(run["angle"][0]),
               "actual_angle_std_rad": float(np.std(run["angle"])),
               "old_angle_rmse_deg": baseline["angle_rmse_deg"],
               "same_train_linear_angle_rmse_deg": fitted_linear["angle_rmse_deg"],
               "new_angle_rmse_deg": learned["angle_rmse_deg"],
               "old_velocity_rmse_rad_s": baseline["velocity_rmse_rad_s"],
               "same_train_linear_velocity_rmse_rad_s": fitted_linear["velocity_rmse_rad_s"],
               "new_velocity_rmse_rad_s": learned["velocity_rmse_rad_s"],
               "imu_integral_vs_encoder_angle_rmse_deg": sensor_consistency(FILES[name]),
               "old_first_second_disturbance_Nm": old_disturbance(FILES[name])}
        rows.append(row)
        ax.plot(run["time"], np.rad2deg(run["angle"] - run["angle"][0]),
                color="black", lw=1, label="measured")
        for idx, (t, actual, prediction) in enumerate(learned["traces"]):
            ax.plot(t, np.rad2deg(prediction - run["angle"][0]),
                    color="tab:orange", lw=1.2, label="friction model" if idx == 0 else None)
        ax.set(title=f"{name} ({row['partition']})", xlabel="time (s)",
               ylabel="yaw from start (deg)")
        ax.grid(alpha=0.25)
        if name == "low004_a":
            ax.legend()
    fig.savefig(OUT / "yaw_cross_frequency_model.png", dpi=160)
    plt.close(fig)
    with (OUT / "run_metrics.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    result = {"model": "J*q_ddot=tau(t-0.003)-B*q_dot-Fc*sign(q_dot)-d-K*(q-4.55), with static lock |net torque|<=Fs at near-zero speed",
              "input": "measured motor-current-derived torque, not command",
              "angle_reference_rad": REFERENCE_ANGLE,
              "parameters": params,
              "same_train_linear_model": {"parameters": linear,
                                          "training_objective": objective(runs, linear)},
              "training_runs": TRAIN,
              "held_out_runs": HOLDOUT,
              "fit_objective": "equal weight per training run; 2 s rolling angle RMSE normalised by motion scale, plus 0.25 times normalised velocity RMSE",
              "search_trace": search_trace,
              "leave_one_training_run_out_parameters": sensitivity,
              "runs": rows}
    (OUT / "fit.json").write_text(json.dumps(result, indent=2) + "\n")
    held = [row for row in rows if row["partition"] == "held_out"]
    labels = [row["run"] for row in held]
    x = np.arange(len(labels))
    fig, ax = plt.subplots(figsize=(10, 5), constrained_layout=True)
    width = 0.26
    ax.bar(x - width, [r["old_angle_rmse_deg"] for r in held], width,
           label="original J/B + first-second offset")
    ax.bar(x, [r["same_train_linear_angle_rmse_deg"] for r in held], width,
           label="linear, same training runs")
    ax.bar(x + width, [r["new_angle_rmse_deg"] for r in held], width,
           label="stick-slip, same training runs")
    ax.set_xticks(x, labels)
    ax.set_ylabel("mean 2 s yaw-angle RMSE (deg)")
    ax.set_title("Complete-run holdout, measured torque to yaw angle")
    ax.grid(axis="y", alpha=0.25)
    ax.legend()
    fig.savefig(OUT / "heldout_model_comparison.png", dpi=160)
    plt.close(fig)
    plot_excitation_evidence()
    print(json.dumps({"parameters": params, "runs": rows}, indent=2))


if __name__ == "__main__":
    main()
