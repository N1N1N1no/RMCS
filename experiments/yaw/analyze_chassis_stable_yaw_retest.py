#!/usr/bin/env python3
"""Analyze the chassis-enabled C-car yaw runs with zero explicit input delay."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

import fit_yaw_plant as plant


ROOT = Path(__file__).resolve().parent
RUNS = {
    "0p5": ROOT / "chassis_stable_yaw_retest_2026-10-13_attempt4",
    "0p7": ROOT / "chassis_stable_yaw_retest_2026-10-13_0p7nm_attempt2",
}
V0 = 0.04  # rad/s, fixed from the earlier exploratory analysis


def select(obs: dict, cycles: range) -> np.ndarray:
    return np.isin(obs["cycle"], list(cycles))


def score(obs: dict, predicted: np.ndarray, selected: np.ndarray) -> dict:
    actual = obs["acceleration"][selected]
    error = actual - predicted[selected]
    return {
        "samples": int(selected.sum()),
        "acceleration_r2": float(1.0 - np.mean(error ** 2) / np.var(actual)),
        "acceleration_rmse_rad_s2": float(np.sqrt(np.mean(error ** 2))),
    }


def linear_params(obs: dict) -> tuple[dict, np.ndarray]:
    coeff = plant.fit(obs, select(obs, range(1, 6)))
    return plant.parameters(coeff), coeff


def fixed_linear_score(obs: dict, params: dict) -> dict:
    a = 1.0 / params["J_kg_m2"]
    b = -params["B_Nm_s_per_rad"] / params["J_kg_m2"]
    base = a * obs["torque"] + b * obs["speed"]
    offset = float(np.mean((obs["acceleration"] - base)[select(obs, range(2, 4))]))
    return {
        **score(obs, base + offset, select(obs, range(4, 8))),
        "disturbance_Nm_calibrated_on_cycles_2_3": float(-offset / a),
    }


def features(obs: dict, friction: bool) -> np.ndarray:
    columns = [obs["torque"], obs["speed"]]
    if friction:
        columns.append(np.tanh(obs["speed"] / V0))
    return np.column_stack(columns)


def rollout(path: Path, params: dict, disturbance: float) -> dict:
    run = plant.load(path)
    start = int(np.searchsorted(run["time"], 4 * plant.CYCLE_SECONDS))
    end = len(run["time"]) - 1
    speed = float(run["speed"][start])
    angle = float(run["angle"][start])
    predicted_speed = np.empty(end - start)
    predicted_angle = np.empty(end - start)
    for output_index, i in enumerate(range(start, end)):
        acceleration = (
            run["torque"][i] - params["B_Nm_s_per_rad"] * speed
            - params["F_c_Nm"] * np.tanh(speed / V0) - disturbance
        ) / params["J_kg_m2"]
        dt = run["time"][i + 1] - run["time"][i]
        speed += dt * acceleration
        angle += dt * speed
        predicted_speed[output_index] = speed
        predicted_angle[output_index] = angle
    return {
        "speed_rmse_rad_s": float(np.sqrt(np.mean(
            (predicted_speed - run["speed"][start + 1:end + 1]) ** 2))),
        "angle_rmse_deg": float(np.rad2deg(np.sqrt(np.mean(
            (predicted_angle - run["angle"][start + 1:end + 1]) ** 2)))),
        "max_abs_angle_error_deg": float(np.rad2deg(np.max(np.abs(
            predicted_angle - run["angle"][start + 1:end + 1])))),
    }


def combined_fit(observations: dict, friction: bool) -> dict:
    names = ("0p5", "0p7")
    width = 3 if friction else 2
    blocks, targets = [], []
    for index, name in enumerate(names):
        obs = observations[name]["run1"]
        mask = select(obs, range(1, 6))
        nuisance = np.zeros((int(mask.sum()), 2))
        nuisance[:, index] = 1.0
        blocks.append(np.column_stack((features(obs, friction)[mask], nuisance)))
        targets.append(obs["acceleration"][mask])
    coeff = np.linalg.lstsq(np.vstack(blocks), np.concatenate(targets), rcond=None)[0]
    slopes, offsets = coeff[:width], coeff[width:]
    params = {
        "J_kg_m2": float(1.0 / slopes[0]),
        "B_Nm_s_per_rad": float(-slopes[1] / slopes[0]),
        "F_c_Nm": float(-slopes[2] / slopes[0]) if friction else 0.0,
        "v0_rad_s": V0 if friction else None,
        "training_disturbance_Nm": {
            name: float(-offsets[index] / slopes[0]) for index, name in enumerate(names)
        },
    }
    metrics = {}
    for index, name in enumerate(names):
        first = observations[name]["run1"]
        second = observations[name]["run2"]
        prediction_first = features(first, friction) @ slopes + offsets[index]
        base_second = features(second, friction) @ slopes
        calibration = select(second, range(2, 4))
        offset_second = float(np.mean(second["acceleration"][calibration]
                                      - base_second[calibration]))
        metrics[name] = {
            "training_run1_cycles_1_5": score(first, prediction_first,
                                               select(first, range(1, 6))),
            "holdout_run1_cycles_6_7": score(first, prediction_first,
                                              select(first, range(6, 8))),
            "holdout_run2_cycles_4_7": score(second, base_second + offset_second,
                                              select(second, range(4, 8))),
            "run2_disturbance_Nm_calibrated_on_cycles_2_3": float(
                -offset_second / slopes[0]),
            "run2_rollout_cycles_4_7": rollout(
                RUNS[name] / "run2.csv", params, float(-offset_second / slopes[0])),
        }
    return {"parameters": params, "scores": metrics}


def main() -> None:
    observations = {}
    manifests = {}
    for name, directory in RUNS.items():
        manifests[name] = json.loads((directory / "manifest.json").read_text())
        if len(manifests[name]) != 2 or not all(item["completed"] for item in manifests[name]):
            raise RuntimeError(f"Incomplete runs for {name}")
        observations[name] = {
            label: plant.observations(plant.load(directory / f"{label}.csv"), 0)
            for label in ("run1", "run2")
        }
    per_amplitude = {}
    for name, pair in observations.items():
        first, second = pair["run1"], pair["run2"]
        params, coeff = linear_params(first)
        independent, _ = linear_params(second)
        independent_score = fixed_linear_score(second, params)
        per_amplitude[name] = {
            "run1_fit_cycles_1_5": params,
            "run2_independent_fit_cycles_1_5": independent,
            "run1_training": plant.score(first, coeff, select(first, range(1, 6))),
            "run1_holdout_cycles_6_7": plant.score(first, coeff,
                                                    select(first, range(6, 8))),
            "run2_holdout_cycles_4_7": independent_score,
            "run2_rollout_cycles_4_7": rollout(
                RUNS[name] / "run2.csv", {**params, "F_c_Nm": 0.0},
                independent_score["disturbance_Nm_calibrated_on_cycles_2_3"]),
        }
    cross = {
        source + "_on_" + target: fixed_linear_score(observations[target]["run2"],
                                                       per_amplitude[source]["run1_fit_cycles_1_5"])
        for source, target in (("0p5", "0p7"), ("0p7", "0p5"))
    }
    result = {
        "model_convention": "no explicit input delay; measured torque and relative yaw angle",
        "chassis_controllers": "original C-car chassis/suspension/wheel/joint components enabled",
        "amplitudes_Nm": {"0p5": 0.5, "0p7": 0.7},
        "manifests": manifests,
        "linear_per_amplitude": per_amplitude,
        "linear_cross_amplitude": cross,
        "joint_linear": combined_fit(observations, False),
        "joint_smooth_friction_exploratory": combined_fit(observations, True),
        "limitations": [
            "zero explicit delay is a model convention, not a measured zero physical latency",
            "chassis pitch, roll, joint angles and integrated yaw were monitored; absolute x/y position was not measured",
            "v0=0.04 rad/s was selected from earlier exploratory data, not independently identified",
            "run-specific constant disturbance is calibrated on cycles 2-3 for independent run scores",
            "acceleration R2 does not by itself validate closed-loop feedforward or angle trajectories",
        ],
    }
    output = ROOT / "chassis_stable_yaw_retest_2026-10-13_analysis.json"
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(f"Wrote {output}")
    for name, item in per_amplitude.items():
        print(name, item["run1_fit_cycles_1_5"],
              "run1_holdout_R2", item["run1_holdout_cycles_6_7"]["acceleration_r2"],
              "run2_holdout_R2", item["run2_holdout_cycles_4_7"]["acceleration_r2"])
    print("joint linear", result["joint_linear"]["parameters"])
    print("joint friction", result["joint_smooth_friction_exploratory"]["parameters"])


if __name__ == "__main__":
    main()
