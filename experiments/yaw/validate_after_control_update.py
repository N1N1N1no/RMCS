#!/usr/bin/env python3
"""Validate the original torque-to-yaw plant under the final PID/FF configuration."""

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import fit_yaw_plant as plant


ROOT = Path(__file__).resolve().parent
DATA = ROOT / "validation_2026-10-02"
FILES = [DATA / f"plant_after_pid_ff_{i:02d}.csv" for i in (1, 2, 3)]
OLD = json.loads((ROOT / "yaw_plant_fit.json").read_text())
J0 = OLD["parameters"]["J_kg_m2"]
B0 = OLD["parameters"]["B_Nm_s_per_rad"]
DELAY = OLD["delay_ms"]


def fixed_coefficients(obs, disturbance_selection):
    """Hold J,B fixed; estimate only this run's constant disturbance."""
    a = 1.0 / J0
    b = -B0 / J0
    c = np.mean(obs["acceleration"][disturbance_selection]
                - a * obs["torque"][disturbance_selection]
                - b * obs["speed"][disturbance_selection])
    return np.array([a, b, c])


def pooled_fit(observations, cycle_draws=None):
    xs, ys = [], []
    for run_index, obs in enumerate(observations):
        if cycle_draws is None:
            chosen = np.isin(obs["cycle"], [1, 2])
            indices = np.flatnonzero(chosen)
        else:
            indices = np.concatenate([np.flatnonzero(obs["cycle"] == cycle)
                                      for cycle in cycle_draws[run_index]])
        n = len(indices)
        indicator = np.tile(np.eye(len(observations))[run_index], (n, 1))
        xs.append(np.column_stack([obs["torque"][indices], obs["speed"][indices], indicator]))
        ys.append(obs["acceleration"][indices])
    x, y = np.concatenate(xs), np.concatenate(ys)
    coefficients = np.linalg.lstsq(x, y, rcond=None)[0]
    return coefficients, float(1.0 - np.mean((y - x @ coefficients) ** 2) / np.var(y))


def main():
    runs = [plant.load(path) for path in FILES]
    obs = [plant.observations(run, DELAY) for run in runs]
    records = []
    curves = []
    for file, run, measurement in zip(FILES, runs, obs):
        first_cycle = measurement["cycle"] == 1
        second_cycle = measurement["cycle"] == 2
        complete = first_cycle | second_cycle
        if first_cycle.sum() < 80 or second_cycle.sum() < 80:
            raise ValueError(f"Missing complete validation cycles: {file}")
        fitted = plant.fit(measurement, complete)
        new_parameters = plant.parameters(fitted)
        fixed = fixed_coefficients(measurement, first_cycle)
        second_score = plant.score(measurement, fixed, second_cycle)
        fixed_all = fixed_coefficients(measurement, complete)
        old_all_score = plant.score(measurement, fixed_all, complete)
        new_all_score = plant.score(measurement, fitted, complete)
        index = np.searchsorted(run["time"], plant.CYCLE_SECONDS * 2)
        stop = min(np.searchsorted(run["time"], plant.CYCLE_SECONDS * 3), len(run["time"]) - 1)
        predicted_speed, predicted_angle = plant.simulate(
            run, index, stop,
            {"J_kg_m2": J0, "B_Nm_s_per_rad": B0,
             "disturbance_Nm": -fixed[2] * J0},
            DELAY,
        )
        records.append({
            "file": str(file),
            "duration_s": float(run["time"][-1]),
            "samples": len(run["time"]),
            "last_phase": int(run["phase"][-1]),
            "start_angle_rad": float(run["angle"][0]),
            "relative_end_angle_rad": float(run["angle"][-1] - run["angle"][0]),
            "max_encoder_speed_rad_s": float(np.max(np.abs(run["encoder_speed"]))),
            "max_command_torque_Nm": float(np.max(np.abs(run["command"]))),
            "fitted_parameters": new_parameters,
            "new_fit_cycles_1_2": new_all_score,
            "old_fixed_J_B_cycles_1_2_with_run_disturbance": old_all_score,
            "old_fixed_J_B_holdout_cycle_2": second_score,
            "holdout_disturbance_Nm_from_cycle_1": float(-fixed[2] * J0),
            "holdout_cycle_2_angle_rmse_rad": float(np.sqrt(np.mean(
                (predicted_angle - run["angle"][index:stop]) ** 2))),
            "holdout_cycle_2_speed_rmse_rad_s": float(np.sqrt(np.mean(
                (predicted_speed - run["speed"][index:stop]) ** 2))),
        })
        curves.append((measurement["time"][second_cycle],
                       measurement["acceleration"][second_cycle],
                       (plant.matrix(measurement) @ fixed)[second_cycle]))

    coefficients, pooled_r2 = pooled_fit(obs)
    pooled = {"J_kg_m2": float(1 / coefficients[0]),
              "B_Nm_s_per_rad": float(-coefficients[1] / coefficients[0]),
              "acceleration_r2": pooled_r2}
    rng = np.random.default_rng(2026)
    boot = []
    change_boot = []
    original_obs = plant.observations(
        plant.load(ROOT / OLD["training_file"]), DELAY)
    matched_old_cycles = np.isin(original_obs["cycle"], [1, 2])
    matched_old = plant.parameters(plant.fit(original_obs, matched_old_cycles))
    for _ in range(5000):
        draws = [rng.choice([1, 2], size=2) for _ in obs]
        candidate, _ = pooled_fit(obs, draws)
        if candidate[0] > 0:
            j_new, b_new = 1 / candidate[0], -candidate[1] / candidate[0]
            boot.append((j_new, b_new))
            old_cycles = rng.integers(1, 6, size=5)
            old_indices = np.concatenate([
                np.flatnonzero(original_obs["cycle"] == cycle) for cycle in old_cycles
            ])
            old_parameters = plant.parameters(plant.fit(original_obs, old_indices))
            change_boot.append((
                100 * (j_new / old_parameters["J_kg_m2"] - 1),
                100 * (b_new / old_parameters["B_Nm_s_per_rad"] - 1),
            ))
    bounds = np.quantile(boot, [0.025, 0.975], axis=0)
    change_bounds = np.quantile(change_boot, [0.025, 0.975], axis=0)
    result = {
        "model": OLD["model"],
        "test_context": "final yaw PID 12/11 and feedforward B/J enabled, but direct motor torque override bypasses controller output",
        "input": OLD["input"],
        "sample_period_s": 0.001,
        "delay_ms_fixed_from_original": DELAY,
        "original_parameters": {"J_kg_m2": J0, "B_Nm_s_per_rad": B0},
        "original_matched_cycles_1_2_parameters": {
            "J_kg_m2": matched_old["J_kg_m2"],
            "B_Nm_s_per_rad": matched_old["B_Nm_s_per_rad"],
        },
        "original_cycle_bootstrap_95pct": OLD["cycle_bootstrap_95pct"],
        "new_pooled_parameters": pooled,
        "new_cycle_bootstrap_95pct_conditional_on_three_runs": {
            "J_kg_m2": bounds[:, 0].tolist(),
            "B_Nm_s_per_rad": bounds[:, 1].tolist(),
        },
        "new_pooled_relative_change_pct": {
            "J": float(100 * (pooled["J_kg_m2"] / J0 - 1)),
            "B": float(100 * (pooled["B_Nm_s_per_rad"] / B0 - 1)),
        },
        "new_vs_original_matched_cycles_1_2_change_pct": {
            "J": float(100 * (pooled["J_kg_m2"] / matched_old["J_kg_m2"] - 1)),
            "B": float(100 * (pooled["B_Nm_s_per_rad"] / matched_old["B_Nm_s_per_rad"] - 1)),
        },
        "relative_change_cycle_bootstrap_95pct": {
            "J_pct": change_bounds[:, 0].tolist(),
            "B_pct": change_bounds[:, 1].tolist(),
        },
        "runs": records,
    }
    lag_scores = []
    for lag in range(7):
        lag_observations = [plant.observations(run, lag) for run in runs]
        _, r2 = pooled_fit(lag_observations)
        lag_scores.append(r2)
    result["new_delay_r2_by_ms_0_to_6"] = lag_scores
    result["new_best_delay_ms"] = int(np.argmax(lag_scores))
    (DATA / "validation_result.json").write_text(json.dumps(result, indent=2) + "\n")

    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    for i, (time, measured, predicted) in enumerate(curves):
        ax = axes.flat[i]
        ax.plot(time, measured, label="measured", linewidth=1)
        ax.plot(time, predicted, label="original model", linewidth=1)
        ax.set_title(f"New run {i + 1}: held-out cycle 2")
        ax.set_xlabel("time (s)")
        ax.set_ylabel("relative yaw acceleration (rad/s²)")
        ax.grid(alpha=0.25)
        if i == 0:
            ax.legend()
    ax = axes.flat[3]
    for x, key, unit in [(0, "J_kg_m2", "J (kg m²)"), (1, "B_Nm_s_per_rad", "B (N m s/rad)")]:
        old_value = J0 if x == 0 else B0
        old_bounds = OLD["cycle_bootstrap_95pct"][key]
        new_value = pooled[key]
        new_bounds = bounds[:, x]
        # Normalize each parameter to the old point estimate to share one axis.
        ax.errorbar(x - 0.1, old_value / old_value,
                    yerr=np.array([[1 - old_bounds[0] / old_value],
                                   [old_bounds[1] / old_value - 1]]),
                    fmt="o", color="#3465a4", capsize=4,
                    label="original" if x == 0 else None)
        ax.errorbar(x + 0.1, new_value / old_value,
                    yerr=np.array([[new_value / old_value - new_bounds[0] / old_value],
                                   [new_bounds[1] / old_value - new_value / old_value]]),
                    fmt="^", color="#e66100", capsize=4,
                    label="new pooled" if x == 0 else None)
        ax.scatter(x - 0.2, matched_old[key] / old_value,
                   marker="D", s=25, color="#7c4aa5",
                   label="original cycles 1-2" if x == 0 else None)
        for record in records:
            ax.scatter(x + 0.2, record["fitted_parameters"][key] / old_value,
                       s=14, color="#6e9b3d", alpha=0.8)
    ax.axhline(1, color="black", linestyle="--", linewidth=0.8)
    ax.set_xticks([0, 1], ["J / original J", "B / original B"])
    ax.set_ylabel("normalized parameter")
    ax.set_ylim(0.88, 1.14)
    ax.grid(alpha=0.25)
    ax.legend()
    fig.savefig(DATA / "validation_plot.png", dpi=160)
    plt.close(fig)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
