#!/usr/bin/env python3
"""Identify yaw reference-to-encoder closed-loop dynamics from four sweep CSVs.

The fit includes a separate offset and linear drift for each training run. The
fourth run is never used to estimate transfer-function parameters. Its initial
offset is measured before the first step; its drift uses the training-run mean.
"""

import csv
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


HERE = Path(__file__).resolve().parent
FILES = [
    "yaw_sweep_2026-10-01.csv",
    "yaw_repeat_1_2026-10-01.csv",
    "yaw_repeat_2_2026-10-01.csv",
    "yaw_repeat_3_2026-10-01.csv",
]
ACTIVE = "/gimbal/yaw_sweep/active"
TARGET = "/gimbal/yaw_sweep/target_offset"
ANGLE = "/gimbal/yaw/angle"
SAMPLE_PERIOD = 0.005  # 1 kHz executor, ValueCollector write_interval=5.


def read_run(name, stride=1):
    with (HERE / name).open(newline="") as stream:
        rows = [r for r in csv.DictReader(stream) if r[ACTIVE] == "1"]
    if not rows:
        raise ValueError(f"No active sweep in {name}")
    rows = rows[::stride]
    index = np.array([int(r["index"]) for r in rows])
    time = (index - index[0]) * SAMPLE_PERIOD
    command = np.array([float(r[TARGET]) for r in rows])
    angle = np.unwrap(np.array([float(r[ANGLE]) for r in rows]))
    changes = np.flatnonzero(np.r_[True, np.diff(command) != 0])
    return {"name": name, "time": time, "command": command, "angle": angle,
            "changes": changes}


def step_response(time, params, order):
    time = np.maximum(time, 0.0)
    if order == 1:
        return 1.0 - np.exp(-time / params[0])
    omega, damping = params[:2]
    if damping < 1.0 - 1e-3:
        root = math.sqrt(1.0 - damping * damping)
        wd = omega * root
        return 1.0 - np.exp(-damping * omega * time) * (
            np.cos(wd * time) + damping / root * np.sin(wd * time))
    if damping > 1.0 + 1e-3:
        root = math.sqrt(damping * damping - 1.0)
        slow = omega * (damping - root)
        fast = omega * (damping + root)
        return 1.0 - (fast * np.exp(-slow * time)
                      - slow * np.exp(-fast * time)) / (fast - slow)
    return 1.0 - np.exp(-omega * time) * (1.0 + omega * time)


def response(run, params, order):
    delay = params[-1]
    time = run["time"]
    command = run["command"]
    result = np.zeros_like(time)
    for j in run["changes"]:
        change = command[j] - (command[j - 1] if j else 0.0)
        if change:
            result += change * step_response(time - time[j] - delay, params, order)
    return result


def fit_linear_terms(runs, params, order):
    """Eliminate offsets/slopes and shared DC gain by least squares."""
    projected = []
    for run in runs:
        time = run["time"]
        base = np.column_stack((np.ones_like(time), time))
        f = response(run, params, order)
        f_res = f - base @ np.linalg.lstsq(base, f, rcond=None)[0]
        y = run["angle"]
        y_res = y - base @ np.linalg.lstsq(base, y, rcond=None)[0]
        projected.append((base, f, f_res, y, y_res))
    numerator = sum(float(np.dot(p[2], p[4])) for p in projected)
    denominator = sum(float(np.dot(p[2], p[2])) for p in projected)
    gain = numerator / denominator if denominator > 1e-12 else 0.0
    nuisance = []
    squared_error = 0.0
    count = 0
    for base, f, _, y, _ in projected:
        offset, drift = np.linalg.lstsq(base, y - gain * f, rcond=None)[0]
        predicted = offset + drift * base[:, 1] + gain * f
        squared_error += float(np.sum((y - predicted) ** 2))
        count += len(y)
        nuisance.append((float(offset), float(drift)))
    return math.sqrt(squared_error / count), float(gain), nuisance


def valid(params, order):
    if order == 1:
        tau, delay = params
        return 0.008 <= tau <= 1.0 and 0 <= delay <= 0.10
    omega, damping, delay = params
    return 1.0 <= omega <= 200.0 and 0.05 <= damping <= 6.0 and 0 <= delay <= 0.10


def objective(params, runs, order):
    if not valid(params, order):
        return 1e4
    error, gain, _ = fit_linear_terms(runs, params, order)
    return error if 0.1 <= gain <= 2.0 else 1e3 + abs(gain)


def nelder_mead(start, scale, runs, order, max_iter=350):
    simplex = [np.array(start, dtype=float)]
    simplex.extend(simplex[0] + np.eye(len(start))[i] * scale[i]
                   for i in range(len(start)))
    values = [objective(p, runs, order) for p in simplex]
    for _ in range(max_iter):
        indices = np.argsort(values)
        simplex = [simplex[i] for i in indices]
        values = [values[i] for i in indices]
        if max(np.linalg.norm(p - simplex[0]) for p in simplex[1:]) < 1e-7:
            break
        center = np.mean(simplex[:-1], axis=0)
        reflected = center + (center - simplex[-1])
        fr = objective(reflected, runs, order)
        if fr < values[0]:
            expanded = center + 2.0 * (reflected - center)
            fe = objective(expanded, runs, order)
            simplex[-1], values[-1] = (expanded, fe) if fe < fr else (reflected, fr)
        elif fr < values[-2]:
            simplex[-1], values[-1] = reflected, fr
        else:
            contracted = center + 0.5 * ((reflected if fr < values[-1] else simplex[-1]) - center)
            fc = objective(contracted, runs, order)
            if fc < min(fr, values[-1]):
                simplex[-1], values[-1] = contracted, fc
            else:
                for i in range(1, len(simplex)):
                    simplex[i] = simplex[0] + 0.5 * (simplex[i] - simplex[0])
                    values[i] = objective(simplex[i], runs, order)
    best = int(np.argmin(values))
    return simplex[best], values[best]


def identify(runs, order):
    if order == 1:
        starts = [[tau, delay] for tau in (0.025, 0.05, 0.1, 0.2, 0.4)
                  for delay in (0, 0.01, 0.03)]
        scale = [0.02, 0.008]
    else:
        starts = [[omega, zeta, delay] for omega in (8, 15, 25, 40, 70)
                  for zeta in (0.4, 0.8, 1.0, 1.5, 2.5)
                  for delay in (0, 0.015, 0.04)]
        scale = [3.0, 0.2, 0.008]
    starts.sort(key=lambda p: objective(p, runs, order))
    candidates = [nelder_mead(p, scale, runs, order) for p in starts[:5]]
    params, error = min(candidates, key=lambda item: item[1])
    _, gain, nuisance = fit_linear_terms(runs, params, order)
    return params, error, gain, nuisance


def validation(run, params, order, gain, drift):
    f = response(run, params, order)
    early = run["time"] < 1.0
    offset = float(np.mean(run["angle"][early] - drift * run["time"][early]
                           - gain * f[early]))
    predicted = offset + drift * run["time"] + gain * f
    residual = run["angle"] - predicted
    return predicted, float(np.sqrt(np.mean(residual ** 2))), float(np.max(np.abs(residual)))


def main():
    full = [read_run(name) for name in FILES]
    fit_runs = full[:3]
    results = {}
    for order in (1, 2):
        params, training_rmse, gain, nuisance = identify(fit_runs, order)
        training_drift = float(np.mean([drift for _, drift in nuisance]))
        full_pred = []
        _, _, full_nuisance = fit_linear_terms(full[:3], params, order)
        for i, run in enumerate(full):
            if i < 3:
                offset, drift = full_nuisance[i]
                full_pred.append(offset + drift * run["time"]
                                 + gain * response(run, params, order))
            else:
                predicted, _, _ = validation(run, params, order, gain, training_drift)
                full_pred.append(predicted)
        holdout = full[-1]
        residual = holdout["angle"] - full_pred[-1]
        holdout_rmse = float(np.sqrt(np.mean(residual ** 2)))
        holdout_peak = float(np.max(np.abs(residual)))
        results[str(order)] = {
            "parameters": [float(p) for p in params],
            "gain": gain,
            "training_rmse_rad": training_rmse,
            "holdout_rmse_rad": holdout_rmse,
            "holdout_peak_abs_error_rad": holdout_peak,
            "training_drifts_rad_per_s": [float(d) for _, d in nuisance],
            "mean_training_drift_rad_per_s": training_drift,
        }
        results[str(order)]["predictions"] = full_pred

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True, layout="constrained")
    for i, (ax, run) in enumerate(zip(axes, full)):
        zero = float(np.mean(run["angle"][run["time"] < 1.0]))
        ax.step(run["time"], np.rad2deg(run["command"]), where="post",
                color="black", linestyle="--", linewidth=1.0, label="Target offset")
        ax.plot(run["time"], np.rad2deg(run["angle"] - zero),
                color="tab:blue", linewidth=1.4, label="Measured encoder")
        ax.plot(run["time"], np.rad2deg(results["2"]["predictions"][i] - zero),
                color="tab:orange", linewidth=1.1, label="Second-order prediction")
        ax.set_ylabel(f"Run {i + 1}\nOffset (deg)")
        ax.grid(alpha=0.25)
        if i == 3:
            ax.set_title("Held-out validation (not used to estimate model parameters)")
    axes[-1].set_xlabel("Time since test start (s)")
    axes[-1].set_xlim(0, 15)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="outside upper center", ncol=3)
    figure = HERE / "yaw_closed_loop_fit.png"
    fig.savefig(figure, dpi=160)

    for result in results.values():
        result.pop("predictions")
    omega, damping, delay = results["2"]["parameters"]
    gain = results["2"]["gain"]
    root = math.sqrt(damping * damping - 1.0) if damping > 1 else float("nan")
    results["2"]["numerator"] = gain * omega * omega
    results["2"]["denominator"] = [1.0, 2.0 * damping * omega, omega * omega]
    results["2"]["poles_rad_per_s"] = [
        -omega * (damping - root), -omega * (damping + root)] if damping > 1 else []
    results["2"]["holdout_rmse_deg"] = math.degrees(results["2"]["holdout_rmse_rad"])
    results["2"]["holdout_peak_abs_error_deg"] = math.degrees(
        results["2"]["holdout_peak_abs_error_rad"])
    report = {
        "input": "yaw_sweep target_offset (rad)",
        "output": "yaw encoder angle (rad)",
        "sample_period_s": SAMPLE_PERIOD,
        "training_files": FILES[:3],
        "validation_file": FILES[3],
        "model_1": results["1"],
        "model_2": results["2"],
        "model_1_parameter_order": ["time_constant_s", "delay_s"],
        "model_2_parameter_order": ["natural_frequency_rad_per_s", "damping_ratio", "delay_s"],
        "second_order_holdout_rmse_reduction_vs_first_order_percent":
            100 * (1 - results["2"]["holdout_rmse_rad"]
                   / results["1"]["holdout_rmse_rad"]),
        "figure": figure.name,
    }
    output = HERE / "yaw_closed_loop_fit.json"
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
