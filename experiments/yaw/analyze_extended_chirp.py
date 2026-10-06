#!/usr/bin/env python3
"""Evaluate the fixed yaw model on the 0.3–0.65 and 2.5–3.0 Hz sweeps."""

import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import analyze_large_chirp_validation as base
import fit_yaw_plant as plant


ROOT = Path(__file__).resolve().parent
FOLDERS = [ROOT / "chirp_extended_high_2026-10-02",
           ROOT / "chirp_extended_low_2026-10-02",
           ROOT / "chirp_bridge_2026-10-02"]
OUT = ROOT / "extended_chirp_validation_2026-10-02"


def main():
    OUT.mkdir(exist_ok=True)
    entries = [(row, folder) for folder in FOLDERS
               for row in json.loads((folder / "manifest.json").read_text())]
    analyses = [base.analyze_one(row, folder) for row, folder in entries]
    records = [item[0] for item in analyses]
    old = json.loads((ROOT / "large_chirp_validation_2026-10-02" / "summary.json").read_text())
    references = [row for row in old["runs"]
                  if row["label"] in ("01_high_006", "03_low_013")]
    (OUT / "summary.json").write_text(json.dumps({
        "model_held_fixed": old["model_held_fixed"],
        "working_position_note": "New runs start at yaw approximately 1.4 rad; the previous reference runs started at approximately 2.6 rad, so frequency and position differ.",
        "new_runs": records,
        "previous_reference_runs": references,
    }, indent=2) + "\n")

    with (OUT / "run_metrics.csv").open("w", newline="") as stream:
        fields = ["label", "frequency_start_hz", "frequency_end_hz",
                  "amplitude_rad", "completed", "start_angle_rad",
                  "max_abs_displacement_rad", "max_abs_motor_speed_rad_s",
                  "max_abs_command_torque_Nm", "acceleration_r2",
                  "acceleration_rmse_rad_s2", "posthoc_disturbance_r2"]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for rec in records:
            writer.writerow({
                "label": rec["label"],
                "frequency_start_hz": rec["frequency_range_hz"][0],
                "frequency_end_hz": rec["frequency_range_hz"][1],
                "amplitude_rad": rec["commanded_amplitude_rad"],
                "completed": rec["completed"],
                "start_angle_rad": rec["start_angle_rad"],
                "max_abs_displacement_rad": rec["max_abs_displacement_rad"],
                "max_abs_motor_speed_rad_s": rec["max_abs_motor_speed_rad_s"],
                "max_abs_command_torque_Nm": rec["max_abs_command_torque_Nm"],
                "acceleration_r2": rec["fixed_model_holdout"]["acceleration_r2"],
                "acceleration_rmse_rad_s2": rec["fixed_model_holdout"]["acceleration_rmse_rad_s2"],
                "posthoc_disturbance_r2": rec[
                    "fixed_model_with_posthoc_constant_disturbance"]["acceleration_r2"],
            })

    fig, axes = plt.subplots(2, 2, figsize=(14, 8), constrained_layout=True)
    colors = ["#3465a4" if r["frequency_range_hz"][0] >= 1.0 else "#e66100"
              for r in records]
    x = np.arange(len(records))
    labels = [f"{r['frequency_range_hz'][0]:g}-{r['frequency_range_hz'][1]:g} Hz\n"
              f"{r['commanded_amplitude_rad']:.2f} rad" for r in records]
    r2 = [r["fixed_model_holdout"]["acceleration_r2"] for r in records]
    axes[0, 0].bar(x, r2, color=colors)
    axes[0, 0].axhline(0, color="black", linewidth=.8)
    axes[0, 0].set_xticks(x, labels)
    axes[0, 0].tick_params(axis="x", labelsize=8, labelrotation=25)
    axes[0, 0].set(ylabel="fixed-model acceleration R²",
                   title="Held-out validation; J, B and delay unchanged")
    for i, value in enumerate(r2):
        axes[0, 0].text(i, value + (0.03 if value >= 0 else -0.12),
                        f"{value:.2f}", ha="center", fontsize=9)
    axes[0, 1].scatter([r["commanded_amplitude_rad"] for r in records],
                       [r["max_abs_displacement_rad"] for r in records],
                       c=colors, s=60)
    for i, rec in enumerate(records, 1):
        shift = (6, -17) if rec["label"] == "02_high_004" else (6, 7)
        axes[0, 1].annotate(str(i),
                            (rec["commanded_amplitude_rad"],
                             rec["max_abs_displacement_rad"]),
                            xytext=shift, textcoords="offset points",
                            fontsize=8)
    axes[0, 1].axhline(.18, color="red", linestyle="--", label="angle stop")
    axes[0, 1].set(xlabel="commanded amplitude (rad)",
                   ylabel="actual peak |offset| (rad)",
                   title="New runs: numbers follow bar order; start yaw 1.16-1.41 rad")
    axes[0, 1].legend()
    for ax, label in ((axes[1, 0], "02_high_004"),
                      (axes[1, 1], "02_verylow_011")):
        rec, run, obs, coeff = next(a for a in analyses if a[0]["label"] == label)
        ax.plot(obs["time"], obs["acceleration"], color="#3465a4",
                linewidth=.8, label="measured")
        ax.plot(obs["time"], plant.matrix(obs) @ coeff,
                color="#e66100", linewidth=1, label="fixed model")
        ax.set(title=f"{label}: R²={rec['fixed_model_holdout']['acceleration_r2']:.3f}",
               xlabel="time (s)", ylabel="yaw acceleration (rad/s²)")
        ax.legend()
    for ax in axes.flat:
        ax.grid(alpha=.25)
    fig.savefig(OUT / "extended_chirp_validation.png", dpi=160)
    plt.close(fig)
    for rec in records:
        print(rec["label"], "R2", round(rec["fixed_model_holdout"]["acceleration_r2"], 3),
              "max angle", round(rec["max_abs_displacement_rad"], 4),
              "start angle", round(rec["start_angle_rad"], 3))


if __name__ == "__main__":
    main()
