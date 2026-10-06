#!/usr/bin/env python3
"""Analyze interleaved low/fast C-car yaw Ki A/B records."""

import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from compare_feedforward import DT, load


ROOT = Path(__file__).resolve().parent
SOURCES = [
    ("slow_forward", ROOT / "low_speed_ki_ab_2026-10-02", 1.0),
    ("slow_reverse", ROOT / "low_speed_ki_ab_reverse_2026-10-02", 1.0),
    ("fast", ROOT / "fast_speed_ki_ab_2026-10-02", 0.5),
]
OUT = ROOT / "ki_optimization_2026-10-02"
METRICS = ["moving_rms_error_deg", "moving_iae_per_reversal_rad_s",
           "hold_rms_error_deg", "late_hold_abs_error_deg",
           "peak_abs_speed_rad_s", "feedback_torque_p99_abs_Nm",
           "command_torque_peak_abs_Nm"]


def metrics(data, transition):
    time = data["t"]
    error = data["error"]
    moving = np.zeros(len(time), dtype=bool)
    holding = np.zeros(len(time), dtype=bool)
    late = np.zeros(len(time), dtype=bool)
    for phase in range(2, 9):
        start = phase * 1.5
        moving |= (time >= start) & (time < start + transition)
        holding |= (time >= start + transition) & (time < start + 1.5)
        late |= (time >= start + 1.25) & (time < start + 1.5)
    if min(moving.sum(), holding.sum(), late.sum()) < 100:
        raise ValueError("Incomplete moving or hold intervals")
    return {
        "moving_rms_error_deg": float(np.rad2deg(np.sqrt(np.mean(error[moving] ** 2)))),
        "moving_iae_per_reversal_rad_s": float(np.sum(np.abs(error[moving])) * DT / 7),
        "hold_rms_error_deg": float(np.rad2deg(np.sqrt(np.mean(error[holding] ** 2)))),
        "late_hold_abs_error_deg": float(np.rad2deg(np.median(np.abs(error[late])))),
        "peak_abs_speed_rad_s": float(np.max(np.abs(data["speed"]))),
        "feedback_torque_p99_abs_Nm": float(np.quantile(np.abs(data["measured_torque"]), .99)),
        "command_torque_peak_abs_Nm": float(np.max(np.abs(data["command_torque"]))),
    }


def one_source(name, folder, transition):
    manifest = json.loads((folder / "manifest.json").read_text())
    protocol = json.loads((folder / "protocol.json").read_text())
    if [row["arm"] for row in manifest] != protocol["order"]:
        raise ValueError(f"Run order differs from protocol: {name}")
    common_time = np.arange(0, 14.99, DT)
    baseline_reference = None
    maximum_reference_difference = 0.0
    records = []
    data_by_label = {}
    for row in manifest:
        path = folder / f"{row['label']}.csv"
        data = load(path)
        if not 14.9 <= data["t"][-1] <= 15.1:
            raise ValueError(f"Unexpected run duration: {path}")
        reference = np.interp(common_time, data["t"], data["reference"])
        if baseline_reference is None:
            baseline_reference = reference
        else:
            difference = float(np.max(np.abs(reference - baseline_reference)))
            maximum_reference_difference = max(maximum_reference_difference, difference)
            if difference > .005:
                raise ValueError(f"Reference trajectory differs by {difference} rad: {path}")
        record = {"batch": name, "label": row["label"], "arm": row["arm"],
                  "pair": row["pair"], "csv": str(path),
                  "start_angle_rad": float(np.median(data["angle"][:100])),
                  **metrics(data, transition)}
        records.append(record)
        data_by_label[row["label"]] = data
    pairs = []
    for index in range(0, len(records), 2):
        left, right = records[index:index + 2]
        if {left["arm"], right["arm"]} != {"current", "ki_005"}:
            raise ValueError(f"Pair is not current/candidate: {name} {index}")
        current = left if left["arm"] == "current" else right
        candidate = right if current is left else left
        pairs.append({"batch": name, "pair": index // 2 + 1,
                      "order": f"{left['arm']} then {right['arm']}",
                      "candidate_minus_current": {
                          field: candidate[field] - current[field] for field in METRICS},
                      "start_angle_difference_rad": candidate["start_angle_rad"]
                                                    - current["start_angle_rad"]})
    return {"name": name, "transition_s": transition,
            "max_reference_difference_rad": maximum_reference_difference,
            "records": records, "pairs": pairs, "data": data_by_label}


def aggregate(records, pairs):
    arms = {arm: {field: float(np.mean([r[field] for r in records if r["arm"] == arm]))
                  for field in METRICS} for arm in ("current", "ki_005")}
    delta = {field: float(np.mean([p["candidate_minus_current"][field] for p in pairs]))
             for field in METRICS}
    return {"arms": arms, "candidate_minus_current": delta,
            "all_pairs_moving_rms_improved": all(
                p["candidate_minus_current"]["moving_rms_error_deg"] < 0 for p in pairs),
            "all_pairs_moving_iae_improved": all(
                p["candidate_minus_current"]["moving_iae_per_reversal_rad_s"] < 0
                for p in pairs)}


def main():
    OUT.mkdir(exist_ok=True)
    batches = [one_source(*source) for source in SOURCES]
    low_records = [r for b in batches[:2] for r in b["records"]]
    low_pairs = [p for b in batches[:2] for p in b["pairs"]]
    fast_records = batches[2]["records"]
    fast_pairs = batches[2]["pairs"]
    result = {
        "slow": aggregate(low_records, low_pairs),
        "fast": aggregate(fast_records, fast_pairs),
        "maximum_reference_difference_by_batch_rad": {
            b["name"]: b["max_reference_difference_rad"] for b in batches},
        "runs": low_records + fast_records,
        "pairs": low_pairs + fast_pairs,
    }
    (OUT / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
    with (OUT / "run_metrics.csv").open("w", newline="") as stream:
        fields = ["batch", "label", "arm", "pair", "start_angle_rad", *METRICS, "csv"]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(result["runs"])

    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    for ax, subset, title in ((axes[0, 0], low_pairs, "slow trajectory"),
                              (axes[0, 1], fast_pairs, "fast trajectory")):
        x = np.arange(len(subset))
        y = [p["candidate_minus_current"]["moving_rms_error_deg"] for p in subset]
        ax.bar(x, y, color=["#3465a4" if v < 0 else "#e66100" for v in y])
        ax.axhline(0, color="black", linewidth=.8)
        ax.set_xticks(x, [f"{p['batch']} {p['pair']}" for p in subset], rotation=25)
        ax.set(title=title, ylabel="candidate - current moving RMS (deg)")
        ax.grid(alpha=.25)
    for ax, batch, title in ((axes[1, 0], batches[0], "slow representative"),
                             (axes[1, 1], batches[2], "fast representative")):
        for row in batch["records"][:2]:
            data = batch["data"][row["label"]]
            ax.plot(data["t"], np.rad2deg(data["error"]), linewidth=.8,
                    label=row["arm"])
        ax.set(title=title, xlabel="time (s)", ylabel="yaw angle error (deg)")
        ax.grid(alpha=.25)
        ax.legend()
    fig.savefig(OUT / "ki_ab_comparison.png", dpi=160)
    plt.close(fig)
    for profile in ("slow", "fast"):
        group = result[profile]
        print(profile, "current RMS", group["arms"]["current"]["moving_rms_error_deg"],
              "candidate RMS", group["arms"]["ki_005"]["moving_rms_error_deg"],
              "delta", group["candidate_minus_current"]["moving_rms_error_deg"],
              "all pairs improved", group["all_pairs_moving_rms_improved"])


if __name__ == "__main__":
    main()
