#!/usr/bin/env python3
"""Fit the C-car yaw motor torque -> relative yaw-angle plant.

The robot logs motor-current-derived torque, yaw encoder angle, gimbal IMU yaw
rate, and chassis IMU yaw rate at approximately 1 kHz. We use the difference of
IMU rates as relative yaw speed because the motor's speed feedback is coarse.

Model: J*q_ddot + B*q_dot + d = tau(t - delay).
The zero-frequency angle integrator makes this a second-order torque-to-angle
plant, even though there is no assumed return spring.

Run 1 cycles 1-5 train J, B, and the run-specific disturbance d. Cycle 0 is
excluded as startup transient; cycles 6-7 and all of run 2 are held out.
"""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


HERE = Path(__file__).resolve().parent
RUN_1 = HERE / "yaw_plant_1790857685500.csv"
RUN_2 = HERE / "yaw_plant_1790857941873.csv"
WINDOW_HALF_WIDTH = 10  # 20 ms centred acceleration/torque window
SAMPLE_STRIDE = 5
PHASE_SECONDS = 0.12
CYCLE_SECONDS = 4 * PHASE_SECONDS


def load(path: Path) -> dict[str, np.ndarray]:
    raw = np.genfromtxt(path, delimiter=",", names=True)
    return {
        "time": raw["t_s"],
        "angle": np.unwrap(raw["angle_rad"]),
        "speed": raw["gimbal_imu_rad_s"] - raw["chassis_imu_rad_s"],
        "encoder_speed": raw["velocity_rad_s"],
        "torque": raw["torque_measured_nm"],
        "command": raw["torque_command_nm"],
        "phase": raw["phase"],
    }


def observations(run: dict[str, np.ndarray], delay_samples: int) -> dict[str, np.ndarray]:
    t, speed, torque = run["time"], run["speed"], run["torque"]
    k = WINDOW_HALF_WIDTH
    # Keep enough margin for all candidate lags and the centred window.
    indices = np.arange(50, len(t) - k - 50, SAMPLE_STRIDE)
    acceleration = (speed[indices + k] - speed[indices - k]) / (
        t[indices + k] - t[indices - k]
    )
    mean_speed = np.array([speed[i - k : i + k + 1].mean() for i in indices])
    mean_torque = np.array(
        [torque[i - k - delay_samples : i + k + 1 - delay_samples].mean() for i in indices]
    )
    return {
        "time": t[indices],
        "acceleration": acceleration,
        "speed": mean_speed,
        "torque": mean_torque,
        "cycle": np.floor(t[indices] / CYCLE_SECONDS).astype(int),
    }


def matrix(obs: dict[str, np.ndarray]) -> np.ndarray:
    return np.column_stack((obs["torque"], obs["speed"], np.ones_like(obs["speed"])))


def fit(obs: dict[str, np.ndarray], selected: np.ndarray) -> np.ndarray:
    return np.linalg.lstsq(matrix(obs)[selected], obs["acceleration"][selected], rcond=None)[0]


def parameters(coefficients: np.ndarray) -> dict[str, float]:
    a, b, c = coefficients
    return {
        "J_kg_m2": float(1.0 / a),
        "B_Nm_s_per_rad": float(-b / a),
        "disturbance_Nm": float(-c / a),
        "velocity_time_constant_s": float(-1.0 / b),
        "nonzero_pole_per_s": float(b),
    }


def score(obs: dict[str, np.ndarray], coeff: np.ndarray, selected: np.ndarray) -> dict[str, float]:
    actual = obs["acceleration"][selected]
    predicted = (matrix(obs) @ coeff)[selected]
    residual = actual - predicted
    return {
        "samples": int(len(actual)),
        "acceleration_r2": float(1.0 - np.mean(residual**2) / np.var(actual)),
        "acceleration_rmse_rad_s2": float(np.sqrt(np.mean(residual**2))),
    }


def simulate(
    run: dict[str, np.ndarray], start: int, stop: int, params: dict[str, float], delay: int
) -> tuple[np.ndarray, np.ndarray]:
    t, u = run["time"], run["torque"]
    speed = float(run["speed"][start])
    angle = float(run["angle"][start])
    speeds = np.empty(stop - start)
    angles = np.empty(stop - start)
    for j, i in enumerate(range(start, stop)):
        dt = t[i + 1] - t[i]
        acceleration = (
            u[max(0, i - delay)]
            - params["B_Nm_s_per_rad"] * speed
            - params["disturbance_Nm"]
        ) / params["J_kg_m2"]
        speed += dt * acceleration
        angle += dt * speed
        speeds[j] = speed
        angles[j] = angle
    return speeds, angles


def main() -> None:
    first, second = load(RUN_1), load(RUN_2)
    lag_scores = []
    for lag in range(7):
        candidate = observations(first, lag)
        train = (candidate["cycle"] >= 1) & (candidate["cycle"] <= 5)
        coeff = fit(candidate, train)
        lag_scores.append(score(candidate, coeff, train)["acceleration_r2"])
    delay = int(np.argmax(lag_scores))
    first_obs, second_obs = observations(first, delay), observations(second, delay)
    train = (first_obs["cycle"] >= 1) & (first_obs["cycle"] <= 5)
    holdout = first_obs["cycle"] >= 6
    coeff = fit(first_obs, train)
    params = parameters(coeff)

    # Cycle bootstrap preserves within-cycle correlation. It measures repeat
    # variability, not uncertainty in the motor torque constant or IMU scale.
    rng = np.random.default_rng(2026)
    boot = np.empty((3000, 2))
    for row in boot:
        selected_cycles = rng.integers(1, 6, size=5)
        chosen = np.concatenate(
            [np.flatnonzero(first_obs["cycle"] == cycle) for cycle in selected_cycles]
        )
        fitted = parameters(fit(first_obs, chosen))
        row[:] = fitted["J_kg_m2"], fitted["B_Nm_s_per_rad"]

    independent = parameters(fit(second_obs, np.ones(len(second_obs["time"]), dtype=bool)))
    holdout_start = int(np.searchsorted(first["time"], 6 * CYCLE_SECONDS))
    predicted_speed, predicted_angle = simulate(
        first, holdout_start, len(first["time"]) - 1, params, delay
    )
    observed_speed = first["speed"][holdout_start:-1]
    observed_angle = first["angle"][holdout_start:-1]
    result = {
        "model": "G_tau_to_yaw(s) = exp(-delay*s)/(J*s^2 + B*s)",
        "input": "motor-current-derived output torque (N m)",
        "output": "yaw encoder angle relative to chassis (rad)",
        "relative_speed": "gimbal IMU yaw rate minus chassis IMU yaw rate (rad/s)",
        "training_file": RUN_1.name,
        "independent_file": RUN_2.name,
        "training_cycles": [1, 2, 3, 4, 5],
        "first_run_holdout_cycles": [6, 7],
        "delay_ms": delay,
        "delay_training_r2_by_ms_0_to_6": lag_scores,
        "parameters": params,
        "cycle_bootstrap_95pct": {
            "J_kg_m2": np.quantile(boot[:, 0], [0.025, 0.975]).tolist(),
            "B_Nm_s_per_rad": np.quantile(boot[:, 1], [0.025, 0.975]).tolist(),
        },
        "training": score(first_obs, coeff, train),
        "first_run_holdout": {
            **score(first_obs, coeff, holdout),
            "velocity_rmse_rad_s": float(
                np.sqrt(np.mean((predicted_speed - observed_speed) ** 2))
            ),
            "angle_rmse_rad": float(np.sqrt(np.mean((predicted_angle - observed_angle) ** 2))),
        },
        "independent_run": {
            **score(second_obs, coeff, np.ones(len(second_obs["time"]), dtype=bool)),
            "duration_s": float(second["time"][-1]),
            "ended_early_at_0_12_rad_angle_limit": True,
            "separately_fitted_J_kg_m2": independent["J_kg_m2"],
            "separately_fitted_B_Nm_s_per_rad": independent["B_Nm_s_per_rad"],
            "separately_fitted_disturbance_Nm": independent["disturbance_Nm"],
        },
    }
    (HERE / "yaw_plant_fit.json").write_text(json.dumps(result, indent=2) + "\n")

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), constrained_layout=True)
    m = holdout
    axes[0].plot(first_obs["time"][m], first_obs["acceleration"][m], label="measured")
    axes[0].plot(first_obs["time"][m], (matrix(first_obs) @ coeff)[m], label="model")
    axes[0].set_title("First run: held-out cycles 6-7")
    axes[0].set_ylabel("yaw acceleration (rad/s²)")
    axes[0].legend()
    axes[1].plot(second_obs["time"], second_obs["acceleration"], label="measured")
    axes[1].plot(second_obs["time"], matrix(second_obs) @ coeff, label="model")
    axes[1].set_title("Independent run: stopped by 0.12 rad travel limit")
    axes[1].set_ylabel("yaw acceleration (rad/s²)")
    axes[1].legend()
    axes[2].plot(first["time"][holdout_start:-1], observed_angle - observed_angle[0], label="encoder")
    axes[2].plot(first["time"][holdout_start:-1], predicted_angle - predicted_angle[0], label="model")
    axes[2].set_title("First-run holdout: angle from measured motor torque")
    axes[2].set_xlabel("time (s)")
    axes[2].set_ylabel("relative yaw angle (rad)")
    axes[2].legend()
    fig.savefig(HERE / "yaw_plant_fit.png", dpi=160)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
