#!/usr/bin/env python3
"""Analyze the 2026-10-13 C-car measured-torque yaw retest without delay."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

import evaluate_zero_delay_yaw as old_eval
import fit_yaw_plant as plant


ROOT = Path(__file__).resolve().parent
DATA = ROOT / "zero_delay_plant_retest_2026-10-13_attempt4"
HIGH = ROOT / "zero_delay_plant_retest_2026-10-13_attempt3/run1.csv"
RUN_1, RUN_2 = DATA / "run1.csv", DATA / "run2.csv"
OLD = json.loads((ROOT / "zero_delay_validation_2026-10-13/results.json").read_text())


def fit_run(path: Path, lag: int, cycles: list[int]) -> tuple[dict, np.ndarray, dict]:
    obs = plant.observations(plant.load(path), lag)
    selected = np.isin(obs["cycle"], cycles)
    coeff = plant.fit(obs, selected)
    return plant.parameters(coeff), coeff, plant.score(obs, coeff, selected)


def evaluate(path: Path, lag: int, params: dict, calibration: list[int],
             heldout: list[int], coefficient_override: np.ndarray | None = None) -> dict:
    run = plant.load(path)
    obs = plant.observations(run, lag)
    calibrate = np.isin(obs["cycle"], calibration)
    test = np.isin(obs["cycle"], heldout)
    if not calibrate.any() or not test.any():
        raise ValueError(f"Missing calibration or holdout cycle in {path}")
    coeff = (coefficient_override if coefficient_override is not None
             else old_eval.fixed_coefficients(obs, params, calibrate))
    score = plant.score(obs, coeff, test)
    first = int(np.searchsorted(run["time"], heldout[0] * plant.CYCLE_SECONDS))
    last = min(int(np.searchsorted(run["time"],
                                   (heldout[-1] + 1) * plant.CYCLE_SECONDS)),
               len(run["time"]) - 1)
    speed, angle = plant.simulate(
        run, first, last,
        {**params, "disturbance_Nm": float(-coeff[2] * params["J_kg_m2"])},
        lag,
    )
    return {
        **score,
        "angle_rmse_deg": float(np.rad2deg(np.sqrt(np.mean(
            (angle - run["angle"][first:last]) ** 2)))),
        "speed_rmse_rad_s": float(np.sqrt(np.mean(
            (speed - run["speed"][first:last]) ** 2))),
        "disturbance_Nm": float(-coeff[2] * params["J_kg_m2"]),
        "calibration_cycles": calibration,
        "heldout_cycles": heldout,
    }


def bootstrap(lag: int) -> dict:
    obs = plant.observations(plant.load(RUN_1), lag)
    rng = np.random.default_rng(20261013)
    values = np.empty((3000, 2))
    for row in values:
        cycles = rng.integers(1, 6, size=5)
        selected = np.concatenate([
            np.flatnonzero(obs["cycle"] == cycle) for cycle in cycles
        ])
        fitted = plant.parameters(plant.fit(obs, selected))
        row[:] = fitted["J_kg_m2"], fitted["B_Nm_s_per_rad"]
    return {
        "J_kg_m2": np.quantile(values[:, 0], [0.025, 0.975]).tolist(),
        "B_Nm_s_per_rad": np.quantile(values[:, 1], [0.025, 0.975]).tolist(),
    }


def disturbance_by_cycle(path: Path, params: dict) -> list[dict]:
    obs = plant.observations(plant.load(path), 0)
    out = []
    for cycle in range(8):
        selected = obs["cycle"] == cycle
        if selected.any():
            out.append({
                "cycle": cycle,
                "samples": int(selected.sum()),
                "mean_required_disturbance_Nm": float(np.mean(
                    obs["torque"][selected]
                    - params["J_kg_m2"] * obs["acceleration"][selected]
                    - params["B_Nm_s_per_rad"] * obs["speed"][selected]
                )),
            })
    return out


def main() -> None:
    manifest = json.loads((DATA / "manifest.json").read_text())
    if len(manifest) != 2 or not all(item["completed"] for item in manifest):
        raise ValueError("Both 0.5 N m runs must have completed")
    versions = {}
    new_params, new_coeff, new_train = fit_run(RUN_1, 0, [1, 2, 3, 4, 5])
    old_params = OLD["versions"]["no_explicit_delay"]["parameters"]
    aligned_params = OLD["versions"]["historical_3ms_alignment"]["parameters"]
    for name, params, lag, coeff in (
        ("new_0ms_from_run1", new_params, 0, new_coeff),
        ("old_0ms_from_0p9Nm", old_params, 0, None),
        ("historical_3ms_alignment", aligned_params, 3, None),
    ):
        versions[name] = {
            "parameters": params,
            "alignment_samples": lag,
            "run1_heldout_cycles_6_7": evaluate(
                RUN_1, lag, params, [1, 2, 3, 4, 5], [6, 7], coeff),
            "run2_heldout_cycle_2": evaluate(RUN_2, lag, params, [1], [2]),
            "run2_heldout_cycles_2_7": evaluate(
                RUN_2, lag, params, [1], [2, 3, 4, 5, 6, 7]),
            "run2_heldout_cycles_4_7_after_calibration_2_3": evaluate(
                RUN_2, lag, params, [2, 3], [4, 5, 6, 7]),
            "high_amplitude_heldout_cycle_2": evaluate(
                HIGH, lag, params, [1], [2]),
        }
    run2_params, _, run2_fit = fit_run(RUN_2, 0, [1, 2, 3, 4, 5])
    result = {
        "input": "measured current-derived torque",
        "model": "J*q_ddot + B*q_dot + d_run = tau_meas(t)",
        "new_test_command_amplitude_Nm": 0.5,
        "run_manifest": manifest,
        "high_amplitude_abort_manifest": json.loads((HIGH.parent / "manifest.json").read_text()),
        "training_run1_cycles_1_5": new_train,
        "training_cycle_bootstrap_95pct": bootstrap(0),
        "new_fit_required_disturbance_by_cycle": {
            "run1": disturbance_by_cycle(RUN_1, new_params),
            "run2": disturbance_by_cycle(RUN_2, new_params),
        },
        "independently_fitted_run2_cycles_1_5": {"parameters": run2_params,
                                                  "training": run2_fit},
        "versions": versions,
        "limitations": [
            "0 samples is a model convention, not measured physical zero delay",
            "run1/run2 are at a new yaw position and 0.5 N m, unlike old 0.9 N m training",
            "the high-amplitude run stopped at the 0.12 rad displacement safety limit",
            "cycle bootstrap excludes torque calibration and IMU systematic errors",
        ],
    }
    output = DATA / "analysis.json"
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
