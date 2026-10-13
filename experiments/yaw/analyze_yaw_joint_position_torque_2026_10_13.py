#!/usr/bin/env python3
"""Test whether C-car yaw fits transfer across torque and starting position.

The 0.1-0.2 rad runs use the same four-phase 2.083 Hz torque stimulus as the
frequency runs. All fits use measured torque and relative IMU speed harmonics.
Only completed runs enter the analysis. Training and validation runs are kept
separate; q=2 rad, 0.7 N m has no repeat and is used only as a cross-condition
check or for fitting a candidate subsequently checked elsewhere.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

import analyze_frequency_yaw_retest as sweep
import check_yaw_friction_candidate_2026_10_13 as friction


ROOT = Path(__file__).resolve().parent
NEAR_05 = ROOT / "chassis_stable_yaw_retest_2026-10-13_attempt4"
NEAR_07 = ROOT / "chassis_stable_yaw_retest_2026-10-13_0p7nm_attempt2"
MID_05 = ROOT / "frequency_yaw_retest_2026-10-13_cross_position"
MID_05_REPEAT = ROOT / "frequency_yaw_retest_2026-10-13_cross_position_repeat"
MID_07 = ROOT / "frequency_yaw_retest_2026-10-13_cross_position_0p7nm_high"


def near_record(directory: Path, run: int, command_nm: float) -> dict:
    manifest = json.loads((directory / "manifest.json").read_text())[run - 1]
    path = directory / f"run{run}.csv"
    if not manifest["completed"]:
        raise ValueError(f"Incomplete run: {path}")
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != manifest["sha256"]:
        raise ValueError(f"CSV hash mismatch: {path}")
    raw = np.genfromtxt(path, delimiter=",", names=True)
    frequency = 1.0 / (4 * 0.12)
    time = raw["t_s"]
    # Eight four-phase cycles were commanded. Remove one startup cycle.
    selected = (time >= 1 / frequency) & (time < min(8 / frequency, time[-1]) - 0.02)
    t = time[selected]
    if len(t) < 100:
        raise ValueError(f"Too few samples: {path}")
    tau, _ = sweep.harmonic(t, raw["torque_measured_nm"][selected], frequency)
    relative_speed = raw["gimbal_imu_rad_s"] - raw["chassis_imu_rad_s"]
    speed, speed_noise = sweep.harmonic(t, relative_speed[selected], frequency)
    angle, angle_noise = sweep.harmonic(
        t, np.unwrap(raw["angle_rad"])[selected], frequency)
    return {
        "label": f"{directory.name}/run{run}",
        "path": str(path),
        "sha256": digest,
        "frequency_hz": frequency,
        "start_angle_rad": manifest["start_angle_rad"],
        "torque_command_Nm": command_nm,
        "torque_phasor": tau,
        "speed_phasor": speed,
        "angle_phasor": angle,
        "speed_harmonic_to_residual_std": abs(speed) / speed_noise,
        "angle_harmonic_to_residual_std": abs(angle) / angle_noise,
    }


def mid_records(directory: Path) -> list[dict]:
    records = sweep.load_group((directory,))
    return [r for r in records if r["completed"]
            and r["speed_harmonic_to_residual_std"] >= 3
            and 3 <= r["frequency_hz"] <= 6.25]


def score(records: list[dict], model: dict) -> dict:
    errors = [friction.phasor_error(r, model) for r in records]
    return {"mean_relative_torque_phasor_error_from_imu_speed":
            float(np.mean(errors)), "per_run": dict(zip(
                [r["label"] for r in records], map(float, errors)))}


def angle_score(records: list[dict], model: dict) -> dict:
    errors = [friction.phasor_error(r, model, from_angle=True)
              for r in records]
    return {"mean_relative_torque_phasor_error_from_encoder_angle":
            float(np.mean(errors)), "per_run": dict(zip(
            [r["label"] for r in records], map(float, errors)))}


def fit_fixed_friction(records: list[dict], fc_nm: float) -> dict:
    """Estimate local J/B while transferring F_c from a different position."""
    rows, targets = [], []
    for r in records:
        speed = r["speed_phasor"]
        torque = r["torque_phasor"]
        omega = 2 * np.pi * r["frequency_hz"]
        scale = abs(torque)
        remainder = (torque - (4 / np.pi) * fc_nm * speed / abs(speed)) / scale
        rows.extend(((speed.real / scale, -omega * speed.imag / scale),
                     (speed.imag / scale, omega * speed.real / scale)))
        targets.extend((remainder.real, remainder.imag))
    b, j = np.linalg.lstsq(np.array(rows), np.array(targets), rcond=None)[0]
    return {"J_kg_m2": float(j), "B_Nm_s_per_rad": float(b),
            "F_c_Nm": float(fc_nm)}


def rollout(records: list[dict], model: dict) -> dict:
    return {r["label"]: friction.cycle_rollout(
        Path(r["path"]), 1 / r["frequency_hz"], model)
            for r in records}


def main() -> None:
    near_05 = [near_record(NEAR_05, i, 0.5) for i in (1, 2)]
    near_07 = [near_record(NEAR_07, i, 0.7) for i in (1, 2)]
    if min(r["speed_harmonic_to_residual_std"]
           for r in near_05 + near_07) < 3:
        raise ValueError("Near-position speed harmonic did not meet quality gate")
    mid_05 = mid_records(MID_05)
    mid_05_repeat = mid_records(MID_05_REPEAT)
    mid_07 = mid_records(MID_07)
    if len(mid_05) != 2 or len(mid_05_repeat) != 2 or len(mid_07) != 2:
        raise ValueError("Expected two qualified complete 3.125/6.25 Hz runs per set")

    near_base = sweep.fit(near_05[:1])
    near_high = sweep.fit(near_07[:1])
    mid_base = sweep.fit(mid_05)
    mid_high = sweep.fit(mid_07)
    slope_near = (near_high["B_Nm_s_per_rad"] - near_base["B_Nm_s_per_rad"]) / 0.2
    slope_mid = (mid_high["B_Nm_s_per_rad"] - mid_base["B_Nm_s_per_rad"]) / 0.2
    mid_from_near = {**mid_base, "B_Nm_s_per_rad":
                     mid_base["B_Nm_s_per_rad"] + 0.2 * slope_near}
    near_from_mid = {**near_base, "B_Nm_s_per_rad":
                      near_base["B_Nm_s_per_rad"] + 0.2 * slope_mid}

    near_train = [near_05[0], near_07[0]]
    near_test = [near_05[1], near_07[1]]
    mid_train = mid_05 + mid_07
    near_linear = {**sweep.fit(near_train), "F_c_Nm": 0.0}
    near_friction = friction.fit_friction(near_train)
    mid_linear = {**sweep.fit(mid_train), "F_c_Nm": 0.0}
    mid_friction = friction.fit_friction(mid_train)
    near_friction_from_mid = fit_fixed_friction(near_05[:1],
                                               mid_friction["F_c_Nm"])
    mid_friction_from_near = fit_fixed_friction(mid_05,
                                               near_friction["F_c_Nm"])
    if any(model["B_Nm_s_per_rad"] < 0 or model["F_c_Nm"] < 0
           for model in (near_friction, mid_friction)):
        raise ValueError("Nonphysical local friction fit")

    result = {
        "model_units": {"J": "kg*m^2", "B": "N*m*s/rad", "F_c": "N*m"},
        "quality_gate": "complete runs, relative IMU speed harmonic / residual std >= 3",
        "position_and_amplitude_cells": {
            "near_0p5": [{k: v for k, v in r.items() if k not in (
                "torque_phasor", "speed_phasor", "angle_phasor", "path")}
                for r in near_05],
            "near_0p7": [{k: v for k, v in r.items() if k not in (
                "torque_phasor", "speed_phasor", "angle_phasor", "path")}
                for r in near_07],
            "mid_0p5": [{"label": r["label"], "start_angle_rad": r["start_angle_rad"],
                          "frequency_hz": r["frequency_hz"], "sha256": r["sha256"]}
                         for r in mid_05],
            "mid_0p7": [{"label": r["label"], "start_angle_rad": r["start_angle_rad"],
                          "frequency_hz": r["frequency_hz"], "sha256": r["sha256"]}
                         for r in mid_07],
        },
        "linear_cell_fits": {
            "near_0p5": near_base, "near_0p7": near_high,
            "mid_0p5": mid_base, "mid_0p7": mid_high,
        },
        "amplitude_slope_B_per_Nm": {"near": slope_near, "mid": slope_mid},
        "cross_position_amplitude_transfer": {
            "near_slope_to_mid_0p7": {
                "unshifted_0p5_model": score(mid_07, {**mid_base, "F_c_Nm": 0.0}),
                "transferred_slope_model": score(mid_07, {**mid_from_near, "F_c_Nm": 0.0}),
                "candidate": mid_from_near,
            },
            "mid_slope_to_near_0p7_repeat": {
                "unshifted_0p5_model": score(near_07[1:], {**near_base, "F_c_Nm": 0.0}),
                "transferred_slope_model": score(near_07[1:], {**near_from_mid, "F_c_Nm": 0.0}),
                "candidate": near_from_mid,
            },
        },
        "cross_position_friction_transfer": {
            "near_fc_to_mid_0p7": {
                "training": [r["label"] for r in near_train + mid_05],
                "validation": [r["label"] for r in mid_07],
                "model": mid_friction_from_near,
                "score": score(mid_07, mid_friction_from_near),
                "same_amplitude_mid_0p5_repeat_score":
                    score(mid_05_repeat, mid_friction_from_near),
            },
            "mid_fc_to_near_0p7_repeat": {
                "training": [r["label"] for r in mid_train + near_05[:1]],
                "validation": [near_07[1]["label"]],
                "model": near_friction_from_mid,
                "score": score(near_07[1:], near_friction_from_mid),
                "angle_score": angle_score(near_07[1:], near_friction_from_mid),
                "angle_rollout": rollout(near_07[1:], near_friction_from_mid),
            },
        },
        "friction_candidate": {
            "rule": "tau_1=(B+j*2*pi*f*J)*v_1+(4/pi)*F_c*v_1/abs(v_1)",
            "rollout_rule": "F_c*tanh(v/0.04 rad/s); constant disturbance calibrated on first post-startup cycle",
            "near": {
                "training": [r["label"] for r in near_train],
                "validation": [r["label"] for r in near_test],
                "linear": near_linear, "friction": near_friction,
                "linear_score": score(near_test, near_linear),
                "friction_score": score(near_test, near_friction),
                "linear_angle_score": angle_score(near_test, near_linear),
                "friction_angle_score": angle_score(near_test, near_friction),
                "angle_quality": {r["label"]: {
                    "harmonic_to_residual_std": r["angle_harmonic_to_residual_std"],
                    "derivative_to_imu_speed_gain": float(abs(
                        1j * 2 * np.pi * r["frequency_hz"]
                        * r["angle_phasor"] / r["speed_phasor"])),
                    "derivative_to_imu_speed_phase_deg": float(np.angle(
                        1j * 2 * np.pi * r["frequency_hz"]
                        * r["angle_phasor"] / r["speed_phasor"], deg=True))}
                    for r in near_test},
                "linear_rollout": rollout(near_test, near_linear),
                "friction_rollout": rollout(near_test, near_friction),
            },
            "mid": {
                "training": [r["label"] for r in mid_train],
                "validation": [r["label"] for r in mid_05_repeat],
                "linear": mid_linear, "friction": mid_friction,
                "linear_score": score(mid_05_repeat, mid_linear),
                "friction_score": score(mid_05_repeat, mid_friction),
                "angle_quality": {r["label"]: {
                    "harmonic_to_residual_std": r["angle_harmonic_to_residual_std"],
                    "passes_threshold_3": r["angle_harmonic_to_residual_std"] >= 3}
                    for r in mid_05_repeat},
            },
        },
        "decision": "Amplitude-dependent friction is a research candidate only. Do not replace the selected J/B map: 0.7 N m near-position angle rollout worsens and 2 rad angle harmonics fail quality checks.",
    }
    output = ROOT / "yaw_joint_position_torque_2026-10-13_analysis.json"
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print("Wrote", output)
    print("B slope near/mid", slope_near, slope_mid)
    print("Cross-transfer", {k: {s: round(v[s][
        "mean_relative_torque_phasor_error_from_imu_speed"], 3)
        for s in ("unshifted_0p5_model", "transferred_slope_model")}
        for k, v in result["cross_position_amplitude_transfer"].items()})
    print("Friction transfer", {k: round(v["score"][
        "mean_relative_torque_phasor_error_from_imu_speed"], 3)
        for k, v in result["cross_position_friction_transfer"].items()})
    for key in ("near", "mid"):
        d = result["friction_candidate"][key]
        print(key, "linear", d["linear"], "friction", d["friction"],
              "holdout speed", d["linear_score"]["mean_relative_torque_phasor_error_from_imu_speed"],
              d["friction_score"]["mean_relative_torque_phasor_error_from_imu_speed"])


if __name__ == "__main__":
    main()
