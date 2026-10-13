#!/usr/bin/env python3
"""Explore a no-delay yaw model with smooth Coulomb friction at the new pose.

The 0.5 N m complete run and one cycle of the 0.9 N m safety-stopped run train
the model. Entire later cycles and the second 0.5 N m run remain held out.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

import fit_yaw_plant as plant


ROOT = Path(__file__).resolve().parent
LOW = ROOT / "zero_delay_plant_retest_2026-10-13_attempt4"
HIGH = ROOT / "zero_delay_plant_retest_2026-10-13_attempt3/run1.csv"
V0 = 0.04  # rad/s; illustrative exploratory choice, not independently validated


def features(obs: dict, friction: bool) -> np.ndarray:
    columns = [obs["torque"], obs["speed"]]
    if friction:
        columns.append(np.tanh(obs["speed"] / V0))
    return np.column_stack(columns)


def score(obs: dict, prediction: np.ndarray, selected: np.ndarray) -> dict:
    actual = obs["acceleration"][selected]
    error = actual - prediction[selected]
    return {
        "samples": int(selected.sum()),
        "acceleration_r2": float(1 - np.mean(error**2) / np.var(actual)),
        "acceleration_rmse_rad_s2": float(np.sqrt(np.mean(error**2))),
    }


def predict(obs: dict, slopes: np.ndarray, disturbance: float,
            friction: bool) -> np.ndarray:
    return features(obs, friction) @ slopes + disturbance


def fit(low: dict, high: dict, friction: bool) -> dict:
    low_train = np.isin(low["cycle"], [1, 2, 3, 4, 5])
    high_train = high["cycle"] == 1
    x_low = features(low, friction)[low_train]
    x_high = features(high, friction)[high_train]
    x = np.vstack([
        np.column_stack((x_low, np.ones(len(x_low)), np.zeros(len(x_low)))),
        np.column_stack((x_high, np.zeros(len(x_high)), np.ones(len(x_high)))),
    ])
    y = np.r_[low["acceleration"][low_train], high["acceleration"][high_train]]
    coef = np.linalg.lstsq(x, y, rcond=None)[0]
    slopes, offsets = coef[:-2], coef[-2:]
    params = {
        "J_kg_m2": float(1 / slopes[0]),
        "B_Nm_s_per_rad": float(-slopes[1] / slopes[0]),
        "F_c_Nm": float(-slopes[2] / slopes[0]) if friction else 0.0,
        "v0_rad_s": V0 if friction else None,
        "d_low1_Nm": float(-offsets[0] / slopes[0]),
        "d_high_Nm": float(-offsets[1] / slopes[0]),
    }
    if min(params["J_kg_m2"], params["B_Nm_s_per_rad"]) <= 0 or params["F_c_Nm"] < 0:
        raise ValueError(f"Nonphysical fitted parameters: {params}")
    low2 = plant.observations(plant.load(LOW / "run2.csv"), 0)
    low2_calibration = np.isin(low2["cycle"], [2, 3])
    low2_offset = float(np.mean(low2["acceleration"][low2_calibration]
                                - features(low2, friction)[low2_calibration] @ slopes))
    return {
        "parameters": params,
        "training_low_0p5": score(low, predict(low, slopes, offsets[0], friction), low_train),
        "training_high_0p9_cycle1": score(
            high, predict(high, slopes, offsets[1], friction), high_train),
        "holdout_low1_cycles6_7": score(
            low, predict(low, slopes, offsets[0], friction),
            np.isin(low["cycle"], [6, 7])),
        "holdout_high_cycle2": score(
            high, predict(high, slopes, offsets[1], friction), high["cycle"] == 2),
        "holdout_low2_cycles4_7": score(
            low2, predict(low2, slopes, low2_offset, friction),
            np.isin(low2["cycle"], [4, 5, 6, 7])),
        "low2_disturbance_Nm_from_cycles2_3": float(-low2_offset / slopes[0]),
    }


def main() -> None:
    low = plant.observations(plant.load(LOW / "run1.csv"), 0)
    high = plant.observations(plant.load(HIGH), 0)
    result = {
        "model_friction": "J*q_ddot + B*q_dot + Fc*tanh(q_dot/v0) + d_run = measured_torque(t)",
        "fixed_v0_rad_s": V0,
        "selection_note": "v0 was chosen during exploratory inspection of this data set; holdout scores are descriptive, not unbiased generalization estimates",
        "training": "0.5 N m run1 cycles1-5 plus 0.9 N m stopped run cycle1; separate constant disturbance per run",
        "heldout": "0.5 N m run1 cycles6-7, 0.9 N m stopped run cycle2, 0.5 N m run2 cycles4-7",
        "friction_candidate": fit(low, high, True),
        "same_data_linear_comparator": fit(low, high, False),
    }
    output = LOW / "friction_exploration.json"
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
