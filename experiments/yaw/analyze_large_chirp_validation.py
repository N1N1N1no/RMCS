#!/usr/bin/env python3
"""Evaluate the fixed yaw plant model on larger, held-out chirp records."""

import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import fit_yaw_plant as plant


ROOT = Path(__file__).resolve().parent
DIRS = [ROOT / "large_chirp_2026-10-02", ROOT / "large_chirp_followup_2026-10-02"]
RESULT_DIR = ROOT / "large_chirp_validation_2026-10-02"
OLD = json.loads((ROOT / "yaw_plant_fit.json").read_text())
J = OLD["parameters"]["J_kg_m2"]
B = OLD["parameters"]["B_Nm_s_per_rad"]
DELAY_MS = OLD["delay_ms"]


def load_chirp(path):
    raw = np.genfromtxt(path, delimiter=",", names=True)
    return {
        "time": raw["t_s"],
        "frequency": raw["frequency_hz"],
        "target": raw["target_offset_rad"],
        "angle": np.unwrap(raw["angle_rad"]),
        "speed": raw["gimbal_imu_rad_s"] - raw["chassis_imu_rad_s"],
        "encoder_speed": raw["velocity_rad_s"],
        "torque": raw["torque_measured_nm"],
        "command": raw["torque_command_nm"],
    }


def fixed_coefficients(obs):
    a, b = 1.0 / J, -B / J
    early = obs["time"] < 1.0
    if early.sum() < 50:
        raise ValueError("Insufficient disturbance calibration samples")
    c = np.mean(obs["acceleration"][early] - a * obs["torque"][early]
                - b * obs["speed"][early])
    return np.array([a, b, c])


def rolling_angles(run, disturbance):
    result = []
    for start in (1.0, 3.0, 5.0):
        end = start + 2.0
        if run["time"][-1] < end + 0.03:
            continue
        i = int(np.searchsorted(run["time"], start))
        j = int(np.searchsorted(run["time"], end))
        _, predicted = plant.simulate(
            run, i, j, {"J_kg_m2": J, "B_Nm_s_per_rad": B,
                        "disturbance_Nm": disturbance}, DELAY_MS)
        actual = run["angle"][i:j]
        result.append({"start_s": start, "end_s": end,
                       "angle_rmse_deg": float(np.rad2deg(np.sqrt(
                           np.mean(np.square(predicted - actual)))))})
    return result


def analyze_one(row, directory):
    path = directory / f"{row['label']}.csv"
    run = load_chirp(path)
    obs = plant.observations(run, DELAY_MS)
    coeff = fixed_coefficients(obs)
    eval_mask = (obs["time"] >= 1.0) & (obs["time"] < run["time"][-1] - 0.05)
    score = plant.score(obs, coeff, eval_mask)
    a, b = coeff[:2]
    posthoc_c = float(np.mean(obs["acceleration"][eval_mask]
                              - a * obs["torque"][eval_mask]
                              - b * obs["speed"][eval_mask]))
    posthoc_score = plant.score(obs, np.array([a, b, posthoc_c]), eval_mask)
    frequency = np.interp(obs["time"], run["time"], run["frequency"])
    bins = []
    if row["start_hz"] < 0.4:
        edges = [0.3, 0.4, 0.5, 0.651]
    elif row["start_hz"] < 1.0:
        edges = [0.4, 0.65, 0.9, 1.11]
    elif row["start_hz"] >= 2.5:
        edges = [2.5, 2.75, 3.01]
    else:
        edges = [1.0, 1.5, 2.0, 2.51]
    for lo, hi in zip(edges[:-1], edges[1:]):
        selected = eval_mask & (frequency >= lo) & (frequency < hi)
        if selected.sum() < 30:
            continue
        bins.append({"low_hz": lo, "high_hz": hi,
                     **plant.score(obs, coeff, selected)})
    fitted = plant.parameters(plant.fit(obs, eval_mask))
    windows = rolling_angles(run, float(-coeff[2] * J))
    return {
        "label": row["label"], "csv": str(path),
        "commanded_amplitude_rad": row["amplitude_rad"],
        "frequency_range_hz": [row["start_hz"], row["end_hz"]],
        "completed": row["completed"], "duration_s": row["duration_s"],
        "start_angle_rad": row["start_angle_rad"],
        "max_abs_displacement_rad": row["max_abs_displacement_rad"],
        "max_abs_motor_speed_rad_s": row["max_abs_motor_speed_rad_s"],
        "max_abs_command_torque_Nm": row["max_abs_command_torque_Nm"],
        "max_abs_measured_torque_Nm": float(np.max(np.abs(run["torque"]))),
        "estimated_disturbance_Nm_from_first_second": float(-coeff[2] * J),
        "fixed_model_holdout": score,
        "fixed_model_with_posthoc_constant_disturbance": posthoc_score,
        "frequency_bins": bins,
        "rolling_angle_2s": windows,
        "diagnostic_refit": {"J_kg_m2": fitted["J_kg_m2"],
                             "B_Nm_s_per_rad": fitted["B_Nm_s_per_rad"]},
    }, run, obs, coeff


def main():
    RESULT_DIR.mkdir(exist_ok=True)
    rows = []
    for directory in DIRS:
        rows.extend((row, directory) for row in json.loads(
            (directory / "manifest.json").read_text()))
    analyses = [analyze_one(row, directory) for row, directory in rows]
    records = [a[0] for a in analyses]
    result = {
        "model_held_fixed": {"J_kg_m2": J, "B_Nm_s_per_rad": B,
                             "delay_ms": DELAY_MS},
        "disturbance_estimation": "Only the first 1 s of each new run estimates one constant disturbance; J, B and delay are unchanged",
        "runs": records,
    }
    (RESULT_DIR / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
    with (RESULT_DIR / "run_metrics.csv").open("w", newline="") as stream:
        fields = ["label", "commanded_amplitude_rad", "completed", "duration_s",
                  "start_angle_rad", "max_abs_displacement_rad",
                  "max_abs_motor_speed_rad_s", "max_abs_command_torque_Nm",
                  "acceleration_r2", "acceleration_rmse_rad_s2",
                  "mean_2s_angle_rmse_deg", "diagnostic_J", "diagnostic_B"]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for rec in records:
            writer.writerow({
                "label": rec["label"], "commanded_amplitude_rad": rec["commanded_amplitude_rad"],
                "completed": rec["completed"], "duration_s": rec["duration_s"],
                "start_angle_rad": rec["start_angle_rad"],
                "max_abs_displacement_rad": rec["max_abs_displacement_rad"],
                "max_abs_motor_speed_rad_s": rec["max_abs_motor_speed_rad_s"],
                "max_abs_command_torque_Nm": rec["max_abs_command_torque_Nm"],
                "acceleration_r2": rec["fixed_model_holdout"]["acceleration_r2"],
                "acceleration_rmse_rad_s2": rec["fixed_model_holdout"]["acceleration_rmse_rad_s2"],
                "mean_2s_angle_rmse_deg": (np.mean([w["angle_rmse_deg"] for w in rec["rolling_angle_2s"]])
                                            if rec["rolling_angle_2s"] else float("nan")),
                "diagnostic_J": rec["diagnostic_refit"]["J_kg_m2"],
                "diagnostic_B": rec["diagnostic_refit"]["B_Nm_s_per_rad"],
            })

    fig, axes = plt.subplots(2, 2, figsize=(12, 9), constrained_layout=True)
    low = [a for a in analyses if a[0]["frequency_range_hz"][0] < 1.0]
    for rec, run, _, _ in low:
        axes[0, 0].scatter(rec["commanded_amplitude_rad"],
                           rec["max_abs_displacement_rad"],
                           color="#e66100" if not rec["completed"] else "#3465a4",
                           marker="x" if not rec["completed"] else "o", s=45)
        short = f"{rec['commanded_amplitude_rad']:.2f}"
        if not rec["completed"]:
            short += " stopped"
        shift = ((-75, 10) if rec["commanded_amplitude_rad"] == 0.14 else
                 (-75, -14) if rec["commanded_amplitude_rad"] == 0.15 else (4, 4))
        axes[0, 0].annotate(short,
                            (rec["commanded_amplitude_rad"],rec["max_abs_displacement_rad"]),
                            xytext=shift,textcoords="offset points",fontsize=8)
        axes[0, 1].scatter(rec["commanded_amplitude_rad"],
                           rec["fixed_model_holdout"]["acceleration_r2"],
                           color="#e66100" if not rec["completed"] else "#3465a4",
                           marker="x" if not rec["completed"] else "o", s=45)
    axes[0, 0].axhline(0.18, color="red", linestyle="--", label="safety stop")
    axes[0, 0].set_ylim(0.03, 0.20)
    axes[0, 0].set(xlabel="commanded angle amplitude (rad)", ylabel="actual peak |offset| (rad)")
    axes[0, 1].set(xlabel="commanded angle amplitude (rad)", ylabel="fixed-model acceleration R²")
    axes[0, 0].legend()
    for ax, label in ((axes[1, 0], "03_low_013"), (axes[1, 1], "01_high_006")):
        match = next((item for item in analyses if item[0]["label"] == label), None)
        if match is None:
            continue
        rec, run, _, _ = match
        t = run["time"]
        ax.plot(t, np.rad2deg(run["target"]), "k--", label="target")
        ax.plot(t, np.rad2deg(run["angle"] - run["angle"][0]),
                color="#3465a4", linewidth=1, label="encoder")
        ax.set(title=label, xlabel="time (s)", ylabel="relative yaw (deg)")
        ax.legend()
    for ax in axes.flat:
        ax.grid(alpha=0.25)
    fig.savefig(RESULT_DIR / "large_chirp_validation.png", dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(11, 7), constrained_layout=True)
    for ax, label in zip(axes, ("03_low_013", "01_high_006")):
        match = next((item for item in analyses if item[0]["label"] == label), None)
        if match is None:
            continue
        rec, _, obs, coeff = match
        t = obs["time"]
        ax.plot(t, obs["acceleration"], color="#3465a4", linewidth=.8, label="measured")
        ax.plot(t, plant.matrix(obs) @ coeff, color="#e66100", linewidth=1, label="fixed model")
        ax.set(title=label, xlabel="time (s)", ylabel="yaw acceleration (rad/s²)")
        ax.grid(alpha=0.25)
        ax.legend()
    fig.savefig(RESULT_DIR / "fixed_model_acceleration.png", dpi=160)
    plt.close(fig)
    for rec in records:
        print(rec["label"], "complete", rec["completed"],
              "actual angle", round(rec["max_abs_displacement_rad"], 4),
              "R2", round(rec["fixed_model_holdout"]["acceleration_r2"], 3),
              "2s angle RMSE", [round(w["angle_rmse_deg"], 3) for w in rec["rolling_angle_2s"]],
              "refit J/B", round(rec["diagnostic_refit"]["J_kg_m2"], 4),
              round(rec["diagnostic_refit"]["B_Nm_s_per_rad"], 3))


if __name__ == "__main__":
    main()
