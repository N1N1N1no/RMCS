#!/usr/bin/env python3
"""Check a Coulomb-friction extension against independent frequency and angle runs."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

import analyze_frequency_yaw_retest as sweep
import fit_yaw_plant as plant


ROOT = Path(__file__).resolve().parent
DATA = ROOT / "frequency_yaw_retest_2026-10-13_matched_generalization"
V0 = 0.04  # rad/s, same fixed smooth-transition width as the earlier analysis


def fit_friction(records: list[dict]) -> dict:
    rows, targets = [], []
    for r in records:
        velocity = r["speed_phasor"]
        scale = abs(r["torque_phasor"])
        omega = 2 * np.pi * r["frequency_hz"]
        row = np.array([velocity / scale,
                        (4 / np.pi) * velocity / abs(velocity) / scale,
                        1j * omega * velocity / scale])
        target = r["torque_phasor"] / scale
        rows.extend((row.real, row.imag))
        targets.extend((target.real, target.imag))
    b, fc, j = np.linalg.lstsq(np.array(rows), np.array(targets), rcond=None)[0]
    return {"J_kg_m2": float(j), "B_Nm_s_per_rad": float(b), "F_c_Nm": float(fc)}


def phasor_error(r: dict, model: dict, from_angle: bool = False) -> float:
    omega = 2 * np.pi * r["frequency_hz"]
    velocity = (1j * omega * r["angle_phasor"] if from_angle
                else r["speed_phasor"])
    predicted = ((model["B_Nm_s_per_rad"] + 1j * omega * model["J_kg_m2"])
                 * velocity)
    if model["F_c_Nm"]:
        predicted += (4 / np.pi) * model["F_c_Nm"] * velocity / abs(velocity)
    return float(abs(predicted - r["torque_phasor"]) / abs(r["torque_phasor"]))


def cycle_rollout(path: Path, period: float, model: dict) -> dict:
    run = plant.load(path)
    time = run["time"]
    speed = run["speed"]
    angle = run["angle"]
    torque = run["torque"]
    j = model["J_kg_m2"]
    b = model["B_Nm_s_per_rad"]
    fc = model["F_c_Nm"]

    def friction(velocity: np.ndarray | float) -> np.ndarray | float:
        return fc * np.tanh(velocity / V0)

    # Estimate each model's constant disturbance from the same first whole
    # post-startup cycle; independent later cycles are short-horizon rollouts.
    calibration = np.where((time >= period) & (time < 2 * period))[0]
    start, end = calibration[0], calibration[-1]
    duration = time[end] - time[start]
    disturbance = (np.trapezoid(torque[calibration] - b * speed[calibration]
                                - friction(speed[calibration]), time[calibration])
                   / duration - j * (speed[end] - speed[start]) / duration)
    speed_errors, angle_errors = [], []
    for cycle in range(2, int(time[-1] / period)):
        indices = np.where((time >= cycle * period)
                           & (time < min((cycle + 1) * period, time[-1])))[0]
        if len(indices) < 2:
            continue
        velocity = float(speed[indices[0]])
        yaw = float(angle[indices[0]])
        for index in indices[:-1]:
            dt = time[index + 1] - time[index]
            velocity += dt * (torque[index] - b * velocity
                              - friction(velocity) - disturbance) / j
            yaw += dt * velocity
            speed_errors.append(velocity - speed[index + 1])
            angle_errors.append(yaw - angle[index + 1])
    return {
        "calibrated_disturbance_Nm": float(disturbance),
        "speed_rmse_rad_s": float(np.sqrt(np.mean(np.square(speed_errors)))),
        "angle_rmse_deg": float(np.rad2deg(np.sqrt(np.mean(np.square(angle_errors))))),
    }


def main() -> None:
    records = sweep.load_group((DATA,))
    train = [r for r in records if r["label"].endswith("run1")]
    test = [r for r in records if r["label"].endswith("run2")]
    linear = {**sweep.fit(train), "F_c_Nm": 0.0}
    friction = fit_friction(train)
    if friction["F_c_Nm"] < 0 or friction["B_Nm_s_per_rad"] < 0:
        raise RuntimeError("Fitted friction parameters are not physically plausible")
    candidates = {"linear": linear, "smooth_Coulomb": friction}
    validation = {}
    for r in test:
        period = 1 / r["frequency_hz"]
        entry = {}
        for name, model in candidates.items():
            entry[name] = {
                "relative_torque_phasor_error_from_imu_speed": phasor_error(r, model),
                "relative_torque_phasor_error_from_encoder_angle":
                    phasor_error(r, model, True),
                "short_cycle_rollout": cycle_rollout(DATA / f"{r['label']}.csv",
                                                     period, model),
            }
        validation[r["label"]] = entry
    result = {
        "train": [r["label"] for r in train],
        "test": [r["label"] for r in test],
        "harmonic_friction_rule": "tau1=(B+j*omega*J)*v1+(4/pi)*Fc*v1/abs(v1)",
        "rollout_friction_rule": "Fc*tanh(v/0.04 rad/s)",
        "models": candidates,
        "validation": validation,
        "decision": "Exploratory friction term is not promoted: it worsens qualified 2.083 and 3.125 Hz held-out encoder-angle short-cycle rollouts.",
    }
    output = ROOT / "yaw_friction_candidate_2026-10-13_analysis.json"
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print("Wrote", output)
    print("models", candidates)
    for label, values in validation.items():
        print(label, {name: {"speed_phasor": round(data[
            "relative_torque_phasor_error_from_imu_speed"], 3),
            "angle_rollout_deg": round(data["short_cycle_rollout"]["angle_rmse_deg"], 3)}
                       for name, data in values.items()})


if __name__ == "__main__":
    main()
