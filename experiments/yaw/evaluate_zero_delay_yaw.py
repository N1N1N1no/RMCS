#!/usr/bin/env python3
"""Re-fit the measured-torque yaw plant with no explicit time shift.

This is an offline comparison on archived C-car records. It does not infer a
physical zero delay or change the controller's feedforward gains.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

import fit_yaw_plant as plant


ROOT = Path(__file__).resolve().parent
OUT = ROOT / "zero_delay_validation_2026-10-13"
VALIDATION = [
    ROOT / "validation_2026-10-02" / f"plant_after_pid_ff_{i:02d}.csv"
    for i in (1, 2, 3)
]


def fixed_coefficients(obs: dict, params: dict, calibration: np.ndarray) -> np.ndarray:
    """Estimate only the run-specific constant disturbance on calibration data."""
    a = 1.0 / params["J_kg_m2"]
    b = -params["B_Nm_s_per_rad"] / params["J_kg_m2"]
    c = float(np.mean(obs["acceleration"][calibration]
                      - a * obs["torque"][calibration]
                      - b * obs["speed"][calibration]))
    return np.array([a, b, c])


def evaluate_run(path: Path, params: dict, lag: int,
                 calibration_cycle: int | None, heldout_cycles: list[int],
                 coefficient_override: np.ndarray | None = None) -> dict:
    run = plant.load(path)
    obs = plant.observations(run, lag)
    heldout = np.isin(obs["cycle"], heldout_cycles)
    if heldout.sum() < 50:
        raise ValueError(f"Insufficient complete cycles in {path}")
    if coefficient_override is None:
        calibration = obs["cycle"] == calibration_cycle
        if calibration.sum() < 50:
            raise ValueError(f"Insufficient calibration samples in {path}")
        coeff = fixed_coefficients(obs, params, calibration)
    else:
        coeff = coefficient_override
    metrics = plant.score(obs, coeff, heldout)
    first = int(np.searchsorted(run["time"], heldout_cycles[0] * plant.CYCLE_SECONDS))
    last = min(int(np.searchsorted(run["time"],
                                   (heldout_cycles[-1] + 1) * plant.CYCLE_SECONDS)),
               len(run["time"]) - 1)
    _, angle = plant.simulate(
        run, first, last,
        {**params, "disturbance_Nm": float(-coeff[2] * params["J_kg_m2"])},
        lag,
    )
    return {
        "file": str(path.relative_to(ROOT)),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "calibration_cycle": calibration_cycle,
        "heldout_cycles": heldout_cycles,
        "estimated_disturbance_Nm": float(-coeff[2] * params["J_kg_m2"]),
        **metrics,
        "angle_rmse_deg": float(np.rad2deg(np.sqrt(
            np.mean((angle - run["angle"][first:last]) ** 2)))),
    }


def fit_original(lag: int) -> tuple[dict, dict, np.ndarray]:
    run = plant.load(plant.RUN_1)
    obs = plant.observations(run, lag)
    train = np.isin(obs["cycle"], [1, 2, 3, 4, 5])
    coeff = plant.fit(obs, train)
    return plant.parameters(coeff), plant.score(obs, coeff, train), coeff


def cycle_bootstrap(lag: int) -> dict:
    obs = plant.observations(plant.load(plant.RUN_1), lag)
    rng = np.random.default_rng(2026)
    estimates = np.empty((3000, 2))
    for row in estimates:
        cycles = rng.integers(1, 6, size=5)
        indices = np.concatenate([
            np.flatnonzero(obs["cycle"] == cycle) for cycle in cycles
        ])
        params = plant.parameters(plant.fit(obs, indices))
        row[:] = params["J_kg_m2"], params["B_Nm_s_per_rad"]
    return {
        "J_kg_m2": np.quantile(estimates[:, 0], [0.025, 0.975]).tolist(),
        "B_Nm_s_per_rad": np.quantile(estimates[:, 1], [0.025, 0.975]).tolist(),
    }


def pooled_validation_fit(lag: int) -> dict:
    """Fit a common J/B and a distinct constant disturbance for each new run."""
    matrices, responses = [], []
    for i, path in enumerate(VALIDATION):
        obs = plant.observations(plant.load(path), lag)
        selected = np.isin(obs["cycle"], [1, 2])
        run_indicator = np.zeros((selected.sum(), len(VALIDATION)))
        run_indicator[:, i] = 1
        matrices.append(np.column_stack((obs["torque"][selected],
                                         obs["speed"][selected], run_indicator)))
        responses.append(obs["acceleration"][selected])
    x, y = np.concatenate(matrices), np.concatenate(responses)
    coeff = np.linalg.lstsq(x, y, rcond=None)[0]
    prediction = x @ coeff
    return {
        "J_kg_m2": float(1 / coeff[0]),
        "B_Nm_s_per_rad": float(-coeff[1] / coeff[0]),
        "acceleration_r2": float(1 - np.mean((y - prediction) ** 2) / np.var(y)),
        "samples": len(y),
    }


def main() -> None:
    versions = {}
    for lag, name in ((0, "no_explicit_delay"), (3, "historical_3ms_alignment")):
        params, train_score, train_coeff = fit_original(lag)
        versions[name] = {
            "alignment_samples": lag,
            "parameters": params,
            "training_cycle_bootstrap_95pct": cycle_bootstrap(lag),
            "training": train_score,
            "original_run_holdout": evaluate_run(
                plant.RUN_1, params, lag, None, [6, 7], train_coeff),
            "independent_original_run": evaluate_run(
                plant.RUN_2, params, lag, 1, [2, 3, 4]),
            "independent_after_control_update": [
                evaluate_run(path, params, lag, 1, [2]) for path in VALIDATION
            ],
            "after_control_update_pooled_diagnostic": pooled_validation_fit(lag),
        }
    result = {
        "model": "J*q_ddot + B*q_dot + d_run = measured_motor_torque",
        "input": "current-derived measured torque, not commanded torque",
        "delay_interpretation": "0 samples is a modeling convention; 3 samples is historical data alignment, not measured physical delay",
        "smoothing_half_window_samples": plant.WINDOW_HALF_WIDTH,
        "sample_stride": plant.SAMPLE_STRIDE,
        "training": "first original run, cycles 1-5; cycle 0 excluded",
        "independent_run_scope": "second original run and three runs after controller update; only each run's constant disturbance calibrated on cycle 1",
        "versions": versions,
    }
    OUT.mkdir(exist_ok=True)
    output = OUT / "results.json"
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
