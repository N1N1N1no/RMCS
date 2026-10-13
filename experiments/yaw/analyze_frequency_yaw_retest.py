#!/usr/bin/env python3
"""Lock-in frequency analysis for the C-car measured-torque yaw plant."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

import analyze_chassis_stable_yaw_retest as earlier
import fit_yaw_plant as plant


ROOT = Path(__file__).resolve().parent
DISCOVERY_DIRS = (
    ROOT / "frequency_yaw_retest_2026-10-13_pilot_attempt2",
    ROOT / "frequency_yaw_retest_2026-10-13_high",
    ROOT / "frequency_yaw_retest_2026-10-13_low",
    ROOT / "frequency_yaw_retest_2026-10-13_20hz",
)
VALIDATION_DIRS = (
    ROOT / "frequency_yaw_retest_2026-10-13_validation",
    ROOT / "frequency_yaw_retest_2026-10-13_validation_1p25_0p4",
)
CROSS_POSITION_DIR = ROOT / "frequency_yaw_retest_2026-10-13_cross_position"
CROSS_POSITION_REPEAT_DIR = ROOT / "frequency_yaw_retest_2026-10-13_cross_position_repeat"
CROSS_AMPLITUDE_DIRS = (
    ROOT / "frequency_yaw_retest_2026-10-13_cross_position_0p7nm",
    ROOT / "frequency_yaw_retest_2026-10-13_cross_position_0p7nm_high",
)
OUTPUT = ROOT / "frequency_yaw_retest_2026-10-13_analysis.json"
FIT_LOW_HZ, FIT_HIGH_HZ = 1.25, 10.0
MIN_SPEED_SNR = 3.0
COMPARATORS = {
    "previous_0p5Nm": {"J_kg_m2": 0.11165369034168546,
                       "B_Nm_s_per_rad": 2.5980686485313185},
    "previous_0p7Nm": {"J_kg_m2": 0.09285235610400416,
                       "B_Nm_s_per_rad": 1.349998652800123},
    "historical_0ms": {"J_kg_m2": 0.14706661116421735,
                       "B_Nm_s_per_rad": 1.2868771207729393},
}


def harmonic(t: np.ndarray, y: np.ndarray, frequency_hz: float) -> tuple[complex, float]:
    omega = 2 * np.pi * frequency_hz
    x = np.column_stack((np.ones_like(t), t - t.mean(),
                         np.cos(omega * t), np.sin(omega * t)))
    coeff = np.linalg.lstsq(x, y, rcond=None)[0]
    residual = y - x @ coeff
    return complex(coeff[2], -coeff[3]), float(np.std(residual))


def read_run(directory: Path, manifest: dict) -> dict:
    label = manifest["label"]
    path = directory / f"{label}.csv"
    raw = np.genfromtxt(path, delimiter=",", names=True)
    time = raw["t_s"]
    frequency = manifest["frequency_hz"]
    # Exclude one start-up cycle. For a guard-stopped run, keep only full cycles
    # that ended before the guard fired; never call that run complete.
    full_cycles = (manifest["cycles"] if manifest["completed"] else
                   int(np.floor((time[-1] + 0.005) * frequency)))
    end = min(full_cycles / frequency, time[-1]) - 0.02
    selected = (time >= 1.0 / frequency) & (time < end)
    t = time[selected]
    torque, torque_noise = harmonic(t, raw["torque_measured_nm"][selected], frequency)
    speed_signal = raw["gimbal_imu_rad_s"] - raw["chassis_imu_rad_s"]
    speed, speed_noise = harmonic(t, speed_signal[selected], frequency)
    angle, angle_noise = harmonic(t, np.unwrap(raw["angle_rad"])[selected], frequency)
    impedance = torque / speed
    return {
        "label": label,
        "frequency_hz": float(frequency),
        "completed": manifest["completed"],
        "full_cycles_used_after_startup": full_cycles - 1,
        "guard_stopped_usable_segment": (not manifest["completed"] and full_cycles >= 5),
        "torque_command_Nm": manifest["torque_command_Nm"],
        "start_angle_rad": manifest["start_angle_rad"],
        "max_abs_displacement_rad": manifest["max_abs_displacement_rad"],
        "torque_phasor": torque,
        "speed_phasor": speed,
        "angle_phasor": angle,
        "torque_amplitude_Nm": abs(torque),
        "speed_amplitude_rad_s": abs(speed),
        "angle_amplitude_rad": abs(angle),
        "speed_harmonic_to_residual_std": abs(speed) / speed_noise,
        "angle_harmonic_to_residual_std": abs(angle) / angle_noise,
        "pointwise_B_Nm_s_per_rad": impedance.real,
        "pointwise_J_kg_m2": impedance.imag / (2 * np.pi * frequency),
        "max_chassis_yaw_change_rad": manifest["max_abs_chassis_yaw_delta_rad"],
        "max_chassis_state_change_rad": max(
            manifest["max_abs_chassis_state_change_rad"].values()),
        "sha256": manifest["sha256"],
    }


def public(record: dict) -> dict:
    result = record.copy()
    for key in ("torque_phasor", "speed_phasor", "angle_phasor"):
        result[key] = {"real": record[key].real, "imag": record[key].imag}
    return result


def fit(records: list[dict]) -> dict:
    rows, outputs = [], []
    for item in records:
        torque = item["torque_phasor"]
        speed = item["speed_phasor"]
        omega = 2 * np.pi * item["frequency_hz"]
        norm = abs(torque)
        rows.extend(((speed.real / norm, -omega * speed.imag / norm),
                     (speed.imag / norm, omega * speed.real / norm)))
        outputs.extend((torque.real / norm, torque.imag / norm))
    b_gain, j_gain = np.linalg.lstsq(np.asarray(rows), np.asarray(outputs), rcond=None)[0]
    return {"J_kg_m2": float(j_gain), "B_Nm_s_per_rad": float(b_gain)}


def error(record: dict, parameters: dict) -> dict:
    torque = record["torque_phasor"]
    speed = record["speed_phasor"]
    angle = record["angle_phasor"]
    omega = 2 * np.pi * record["frequency_hz"]
    impedance = parameters["B_Nm_s_per_rad"] + 1j * omega * parameters["J_kg_m2"]
    return {
        "frequency_hz": record["frequency_hz"],
        "label": record["label"],
        "relative_torque_phasor_error_from_imu_speed": abs(impedance * speed - torque) / abs(torque),
        "relative_torque_phasor_error_from_encoder_angle": abs(impedance * 1j * omega * angle - torque) / abs(torque),
    }


def score(records: list[dict], parameters: dict) -> dict:
    per_run = [error(item, parameters) for item in records]
    return {
        "per_run": per_run,
        "mean_relative_torque_phasor_error_from_imu_speed": float(np.mean([
            item["relative_torque_phasor_error_from_imu_speed"] for item in per_run])),
        "mean_relative_torque_phasor_error_from_encoder_angle": float(np.mean([
            item["relative_torque_phasor_error_from_encoder_angle"] for item in per_run])),
    }


def load_group(directories: tuple[Path, ...]) -> list[dict]:
    records = []
    for directory in directories:
        if not (directory / "manifest.json").exists():
            continue
        for manifest in json.loads((directory / "manifest.json").read_text()):
            records.append(read_run(directory, manifest))
    return records


def quality(record: dict) -> bool:
    return ((record["completed"] or record["guard_stopped_usable_segment"])
            and FIT_LOW_HZ <= record["frequency_hz"] <= FIT_HIGH_HZ
            and record["speed_harmonic_to_residual_std"] >= MIN_SPEED_SNR)


def earlier_position_scores(models: dict) -> dict:
    """Hold-out score at the earlier 0.5 and 0.7 N m yaw positions."""
    output = {}
    for amplitude, directory in earlier.RUNS.items():
        obs = plant.observations(plant.load(directory / "run2.csv"), 0)
        output[amplitude] = {}
        for name, params in models.items():
            acceleration = earlier.fixed_linear_score(obs, params)
            rollout = earlier.rollout(
                directory / "run2.csv", {**params, "F_c_Nm": 0.0},
                acceleration["disturbance_Nm_calibrated_on_cycles_2_3"])
            output[amplitude][name] = {
                "holdout_cycles_4_7": acceleration,
                "rollout_cycles_4_7": rollout,
            }
    return output


def main() -> None:
    discovery = load_group(DISCOVERY_DIRS)
    training = [item for item in discovery if item["label"].endswith("run1") and quality(item)]
    selection = [item for item in discovery if item["label"].endswith("run2") and quality(item)]
    if len(training) != 5 or len(selection) != 5:
        raise RuntimeError("Expected five usable frequencies in each discovery repeat")
    candidate = fit(training)
    independent = fit(selection)
    models = {"frequency_fit_run1": candidate, **COMPARATORS}
    selection_scores = {name: score(selection, params) for name, params in models.items()}
    chosen = min(selection_scores, key=lambda name: selection_scores[name][
        "mean_relative_torque_phasor_error_from_imu_speed"])
    sensitivity = {
        item["label"]: fit([other for other in training if other is not item])
        for item in training
    }
    validation = load_group(VALIDATION_DIRS)
    valid = [item for item in validation if quality(item)]
    validation_scores = ({name: score(valid, params) for name, params in models.items()}
                         if validation else None)
    cross_position = load_group((CROSS_POSITION_DIR,))
    cross_position_valid = [item for item in cross_position if quality(item)]
    cross_position_scores = ({name: score(cross_position_valid, params)
                              for name, params in models.items()}
                             if cross_position_valid else None)
    cross_position_repeat = load_group((CROSS_POSITION_REPEAT_DIR,))
    cross_position_repeat_valid = [item for item in cross_position_repeat if quality(item)]
    position_fit = fit(cross_position_valid) if cross_position_valid else None
    position_models = ({"cross_position_0p5_fit": position_fit, **models}
                       if position_fit else models)
    cross_position_repeat_scores = (
        {name: score(cross_position_repeat_valid, params)
         for name, params in position_models.items()}
        if cross_position_repeat_valid else None)
    cross_amplitude = load_group(CROSS_AMPLITUDE_DIRS)
    cross_amplitude_valid = [item for item in cross_amplitude if quality(item)]
    cross_amplitude_scores = (
        {name: score(cross_amplitude_valid, params)
         for name, params in position_models.items()}
        if cross_amplitude_valid else None)
    result = {
        "frequency_definition": "fundamental of + - - + four-phase torque waveform, f=1/(4*phase_seconds)",
        "input": "motor-current-derived measured torque",
        "output": "yaw encoder angle relative to chassis",
        "relative_speed": "gimbal IMU yaw rate minus chassis IMU yaw rate",
        "model": "G_tau_to_yaw(s)=1/(J*s^2+B*s), no explicit pure delay",
        "lockin_method": "fit intercept, linear trend, cosine and sine after the first cycle; compare measured torque and relative-speed phasors",
        "quality_gate": {"fit_band_hz": [FIT_LOW_HZ, FIT_HIGH_HZ],
                         "minimum_speed_harmonic_to_residual_std": MIN_SPEED_SNR},
        "discovery_runs": [public(item) for item in discovery],
        "candidate_fit_on_run1": candidate,
        "independent_fit_on_run2_for_consistency_only": independent,
        "leave_one_frequency_out_sensitivity": sensitivity,
        "selection_scores_run2": selection_scores,
        "selected_model": chosen,
        "selected_model_scope": "discovery yaw position, 0.5 N m only; not a global yaw model",
        "validation_runs": [public(item) for item in validation],
        "validation_scores": validation_scores,
        "earlier_position_holdout_scores": earlier_position_scores(models),
        "cross_position_runs": [public(item) for item in cross_position],
        "cross_position_scores": cross_position_scores,
        "cross_position_0p5_candidate_fit_on_quality_first": position_fit,
        "cross_position_repeat_runs": [public(item) for item in cross_position_repeat],
        "cross_position_repeat_scores": cross_position_repeat_scores,
        "cross_amplitude_0p7_runs": [public(item) for item in cross_amplitude],
        "cross_amplitude_0p7_scores_on_complete_quality_runs": cross_amplitude_scores,
        "final_operating_point_models": [
            {"start_yaw_rad_range": [0.11, 0.12], "torque_command_Nm": 0.7,
             "parameters": COMPARATORS["previous_0p7Nm"],
             "basis": "earlier time-domain independent run"},
            {"start_yaw_rad_range": [0.21, 0.23], "torque_command_Nm": 0.5,
             "parameters": COMPARATORS["previous_0p5Nm"],
             "basis": "earlier time-domain independent run"},
            {"start_yaw_rad_range": [1.97, 2.17], "torque_command_Nm": 0.5,
             "parameters": position_fit,
             "basis": "first moved-position 3.125/6.25 Hz fit; second batch held out"},
            {"start_yaw_rad_range": [4.73, 5.39], "torque_command_Nm": 0.5,
             "parameters": candidate,
             "basis": "first discovery batch 1.25-10 Hz fit; other two batches held out"},
        ],
        "limitations": [
            "20 Hz speed harmonic is near residual noise and is excluded from parameter fitting",
            "0.625 Hz gives unstable/negative pointwise J because of slow drift and too few independent cycles",
            "10 Hz is above the range where a single J/B explains the response well, despite passing the SNR gate",
            "0.5 N m parameters differ between the discovery and moved yaw positions",
            "the 0.7 N m cross-amplitude check shows effective damping changes at the same moved yaw position",
            "the 0.7 N m 2.083 Hz run hit the yaw displacement guard and is not a completed validation run",
            "torque and the two IMUs do not share a calibrated common hardware timestamp",
            "zero explicit delay is a model convention, not a measured physical zero delay",
        ],
    }
    OUTPUT.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print("Candidate", candidate, "independent refit", independent)
    print("Chosen on run2", chosen)
    for name, rec in selection_scores.items():
        print("selection", name, rec["mean_relative_torque_phasor_error_from_imu_speed"])
    if validation_scores:
        for name, rec in validation_scores.items():
            print("validation", name, rec["mean_relative_torque_phasor_error_from_imu_speed"])
    print("Wrote", OUTPUT)


if __name__ == "__main__":
    main()
