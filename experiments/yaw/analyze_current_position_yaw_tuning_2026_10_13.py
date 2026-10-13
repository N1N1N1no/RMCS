#!/usr/bin/env python3
"""Fit current-position yaw J/B on run1 and score independent run2."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

import analyze_frequency_yaw_retest as sweep


ROOT = Path(__file__).resolve().parent
DATA = ROOT / "current_position_yaw_tuning_short_2026-10-13"
COMPARATORS = {
    "previous_best_6p23rad": {"J_kg_m2": 0.12304739162041455,
                               "B_Nm_s_per_rad": 1.931395397790235},
    "previous_5p3rad_0p5Nm": {"J_kg_m2": 0.08015,
                                "B_Nm_s_per_rad": 2.64024},
}


def quality(record: dict) -> dict:
    derivative = 1j * 2 * np.pi * record["frequency_hz"] * record["angle_phasor"]
    ratio = derivative / record["speed_phasor"]
    return {
        "speed_harmonic_to_residual_std": record["speed_harmonic_to_residual_std"],
        "speed_pass": record["speed_harmonic_to_residual_std"] >= 3.0,
        "angle_harmonic_to_residual_std": record["angle_harmonic_to_residual_std"],
        "angle_derivative_to_imu_gain": float(abs(ratio)),
        "angle_derivative_to_imu_phase_deg": float(np.angle(ratio, deg=True)),
        "angle_pass": bool(record["angle_harmonic_to_residual_std"] >= 3.0
                           and abs(abs(ratio) - 1.0) <= 0.25
                           and abs(np.angle(ratio, deg=True)) <= 15.0),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", type=Path, default=DATA)
    args = parser.parse_args()
    data = args.data.resolve()
    manifest = json.loads((data / "manifest.json").read_text())
    if [row["label"] for row in manifest] != [
        "f3p125_run1", "f3p125_run2", "f6p25_run1", "f6p25_run2"
    ] or not all(row["completed"] and row["torque_command_Nm"] == 0.7
                 for row in manifest):
        raise RuntimeError("Expected four complete, 0.7 N m runs")
    records = [sweep.read_run(data, row) for row in manifest]
    train = [row for row in records if row["label"].endswith("run1")]
    holdout = [row for row in records if row["label"].endswith("run2")]
    fitted = sweep.fit(train)
    models = {"current_position_fit_run1": fitted, **COMPARATORS}
    scores = {name: sweep.score(holdout, model) for name, model in models.items()}
    result = {
        "input": "motor-current-derived measured torque",
        "output": "gimbal IMU yaw rate minus chassis IMU yaw rate",
        "start_yaw_span_rad": [min(row["start_angle_rad"] for row in records),
                               max(row["start_angle_rad"] for row in records)],
        "torque_command_Nm": 0.7,
        "frequency_hz": [3.125, 6.25],
        "train": "run1 at both frequencies, after excluding the first cycle",
        "holdout": "run2 at both frequencies, with fixed parameters",
        "records": [{**sweep.public(row), **quality(row)} for row in records],
        "models": models,
        "scores_on_holdout": scores,
        "scope": "local frequency-domain equivalent; no automatic closed-loop validation",
    }
    (data / "analysis.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print("start span", result["start_yaw_span_rad"])
    print("fit", fitted)
    for row in records:
        q = quality(row)
        print(row["label"], "speed SNR", round(q["speed_harmonic_to_residual_std"], 2),
              "angle pass", q["angle_pass"])
    for name, score in scores.items():
        print(name, "holdout speed error",
              score["mean_relative_torque_phasor_error_from_imu_speed"],
              "per freq", [round(r["relative_torque_phasor_error_from_imu_speed"], 3)
                           for r in score["per_run"]])


if __name__ == "__main__":
    main()
