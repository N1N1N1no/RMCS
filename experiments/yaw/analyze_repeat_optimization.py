#!/usr/bin/env python3
"""Summarize interleaved C-car yaw feedforward trials and export figures."""

import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from compare_feedforward import DT, HOLD, TRANSITION, load, metrics


ROOT = Path(__file__).resolve().parent
GROUPS = [
    ("lower_B_higher_J", ROOT / "repeat_2026-10-02"),
    ("higher_B_lower_J", ROOT / "repeat_2026-10-02_opposite"),
    ("higher_B_mild_J", ROOT / "repeat_2026-10-02_refined"),
]
MEASURES = [
    "moving_rms_error_deg",
    "mean_moving_iae_rad_s",
    "median_late_abs_error_deg",
    "measured_torque_p99_abs_Nm",
    "speed_peak_abs_rad_s",
    "command_torque_peak_abs_Nm",
    "dynamic_change_rms_deg",
]


def dynamic_change_rms(data):
    """RMS of error change relative to the 100 ms before each reversal."""
    t, error = data["t"], data["error"]
    changes = []
    for phase in range(2, 9):
        start = phase * HOLD
        before = (t >= start - 0.1) & (t < start)
        moving = (t >= start) & (t < start + TRANSITION)
        if not np.any(before) or not np.any(moving):
            raise ValueError("Incomplete reversal for drift metric")
        changes.extend(error[moving] - np.median(error[before]))
    return float(np.rad2deg(np.sqrt(np.mean(np.square(changes)))))


def summary(values):
    return {"mean": float(np.mean(values)), "sd": float(np.std(values, ddof=1)),
            "min": float(np.min(values)), "max": float(np.max(values))}


def main():
    records = []
    pair_differences = {}
    reference_run = None
    for group_name, directory in GROUPS:
        manifest = json.loads((directory / "manifest.json").read_text())
        if len(manifest) != 6 or [r["arm"] for r in manifest] != ["current", "candidate"] * 3:
            raise ValueError(f"Incomplete interleaved manifest: {directory}")
        group_rows = []
        for row in manifest:
            path = directory / f"{row['label']}.csv"
            data = load(path)
            if abs(len(data["t"]) * DT - 15.0) > 0.1:
                raise ValueError(f"Unexpected sweep duration: {path}")
            if reference_run is None:
                reference_run = data
            else:
                common = np.arange(0.0, 14.99, DT)
                base = np.interp(common, reference_run["t"], reference_run["reference"])
                actual = np.interp(common, data["t"], data["reference"])
                if np.max(np.abs(base - actual)) > 0.005:
                    raise ValueError(f"Reference differs: {path}")
            result = {"group": group_name, "label": row["label"], "arm": row["arm"],
                      "B": row["B"], "J": row["J"], "csv": str(path),
                      **metrics(data), "dynamic_change_rms_deg": dynamic_change_rms(data)}
            result.pop("reversals")
            group_rows.append(result)
            records.append(result)
        pair_differences[group_name] = []
        for pair in range(3):
            current, candidate = group_rows[2 * pair: 2 * pair + 2]
            pair_differences[group_name].append({
                "pair": pair + 1,
                "current_csv": current["csv"],
                "candidate_csv": candidate["csv"],
                "candidate_minus_current": {
                    measure: candidate[measure] - current[measure] for measure in MEASURES
                },
            })

    all_current = [r for r in records if r["arm"] == "current"]
    overview = {
        "protocol": "3 interleaved current/candidate pairs per candidate; 15 s each; 7 full reversals per run",
        "current_9_run_summary": {m: summary([r[m] for r in all_current]) for m in MEASURES},
        "candidates": {},
        "pairs": pair_differences,
        "runs": records,
    }
    for group_name, _ in GROUPS:
        current = [r for r in records if r["group"] == group_name and r["arm"] == "current"]
        candidate = [r for r in records if r["group"] == group_name and r["arm"] == "candidate"]
        overview["candidates"][group_name] = {
            "gains": {"B": candidate[0]["B"], "J": candidate[0]["J"]},
            "current": {m: summary([r[m] for r in current]) for m in MEASURES},
            "candidate": {m: summary([r[m] for r in candidate]) for m in MEASURES},
            "paired_candidate_minus_current": {
                m: summary([p["candidate_minus_current"][m] for p in pair_differences[group_name]])
                for m in MEASURES
            },
        }

    (ROOT / "repeat_optimization_summary.json").write_text(json.dumps(overview, indent=2) + "\n")
    fields = ["group", "label", "arm", "B", "J", "csv", "samples", *MEASURES]
    with (ROOT / "repeat_optimization_runs.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(records)

    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True, constrained_layout=True)
    measures = ["moving_rms_error_deg", "mean_moving_iae_rad_s", "measured_torque_p99_abs_Nm"]
    labels = ["Moving RMS error (deg)", "Mean reversal IAE (rad s)", "Measured torque |P99| (N m)"]
    xlabels = ["B down, J up", "B up, J down", "B up, mild J down"]
    for group_number, (group_name, _) in enumerate(GROUPS):
        pairs = pair_differences[group_name]
        for ax, measure in zip(axes, measures):
            for pair_number, pair in enumerate(pairs):
                current = next(r for r in records if r["csv"] == pair["current_csv"])
                candidate = next(r for r in records if r["csv"] == pair["candidate_csv"])
                jitter = (pair_number - 1) * 0.07
                xx = group_number + jitter
                ax.plot([xx - 0.025, xx + 0.025], [current[measure], candidate[measure]],
                        color="#8c8c8c", alpha=0.7, linewidth=1)
                ax.scatter(xx - 0.025, current[measure], color="#3465a4", marker="o", s=30,
                           label="current" if group_number == pair_number == 0 else None)
                ax.scatter(xx + 0.025, candidate[measure], color="#e66100", marker="^", s=35,
                           label="candidate" if group_number == pair_number == 0 else None)
    for ax, label in zip(axes, labels):
        ax.set_ylabel(label)
        ax.grid(alpha=0.25)
    axes[0].legend()
    axes[-1].set_xticks(range(len(GROUPS)), xlabels)
    fig.savefig(ROOT / "repeat_optimization_comparison.png", dpi=160)
    plt.close(fig)

    for group_name in overview["candidates"]:
        result = overview["candidates"][group_name]
        print(group_name, "gains", result["gains"])
        for measure in ("moving_rms_error_deg", "mean_moving_iae_rad_s", "dynamic_change_rms_deg"):
            print(" ", measure, "current", result["current"][measure]["mean"],
                  "candidate", result["candidate"][measure]["mean"],
                  "paired delta", result["paired_candidate_minus_current"][measure]["mean"])


if __name__ == "__main__":
    main()
