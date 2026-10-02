#!/usr/bin/env python3
"""Verify the common reference and summarize the smooth yaw controller A/B pairs."""

import csv
import hashlib
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from compare_feedforward import DT, load, metrics
from analyze_repeat_optimization import dynamic_change_rms
from run_controller_ab import original_config
from run_repeat_optimization import SOURCE_YAML


ROOT = Path(__file__).resolve().parent
RUN_DIR = ROOT / "controller_ab_2026-10-02"
MEASURES = (
    "moving_rms_error_deg",
    "mean_moving_iae_rad_s",
    "dynamic_change_rms_deg",
    "median_late_abs_error_deg",
    "speed_peak_abs_rad_s",
    "measured_torque_p99_abs_Nm",
    "command_torque_peak_abs_Nm",
)


def summary(values):
    return {"mean": float(np.mean(values)), "sd": float(np.std(values, ddof=1)),
            "min": float(np.min(values)), "max": float(np.max(values))}


def main():
    manifest = json.loads((RUN_DIR / "manifest.json").read_text())
    protocol = json.loads((RUN_DIR / "protocol.json").read_text())
    expected = ["original", "final"] * 3
    if len(manifest) != 6 or [r["arm"] for r in manifest] != expected:
        raise ValueError("Need three complete interleaved original/final pairs")
    if protocol["order"] != expected:
        raise ValueError("Protocol order differs from actual runs")
    original_hash = hashlib.sha256(original_config(SOURCE_YAML.read_text()).encode()).hexdigest()

    records = []
    data_by_label = {}
    reference = None
    common_time = np.arange(0.0, 14.99, DT)
    max_reference_difference = 0.0
    for row in manifest:
        path = RUN_DIR / f"{row['label']}.csv"
        data = load(path)
        if abs(len(data["t"]) * DT - 15.0) > 0.1:
            raise ValueError(f"Unexpected sweep duration: {path}")
        actual_reference = np.interp(common_time, data["t"], data["reference"])
        if reference is None:
            reference = actual_reference
        else:
            discrepancy = float(np.max(np.abs(reference - actual_reference)))
            max_reference_difference = max(max_reference_difference, discrepancy)
            if discrepancy > 0.005:
                raise ValueError(f"Reference waveform differs by {discrepancy:.4f} rad: {path}")
        target_scale = 0.0 if row["arm"] == "original" else 1.0
        if not np.allclose(data["scale"], target_scale, atol=0.01):
            raise ValueError(f"Unexpected feedforward scale: {path}")
        expected_hash = (protocol["source_final_yaml_sha256"] if row["arm"] == "final"
                         else protocol.get("original_yaml_sha256", original_hash))
        if row["config_sha256"] != expected_hash:
            raise ValueError(f"Configuration hash mismatch: {path}")
        m = metrics(data)
        records.append({"label": row["label"], "arm": row["arm"],
                        "pair": row["pair"], "csv": str(path),
                        **{k: v for k, v in m.items() if k != "reversals"},
                        "dynamic_change_rms_deg": dynamic_change_rms(data),
                        "start_angle_rad": float(np.median(data["angle"][:100])),
                        "end_angle_rad": float(np.median(data["angle"][-100:]))})
        data_by_label[row["label"]] = data

    pairs = []
    for i in range(3):
        original, final = records[2 * i:2 * i + 2]
        pairs.append({"pair": i + 1,
                      "original_csv": original["csv"], "final_csv": final["csv"],
                      "final_minus_original": {
                          measure: final[measure] - original[measure] for measure in MEASURES
                      }})
    result = {
        "protocol": protocol,
        "maximum_reference_difference_rad": max_reference_difference,
        "original": {m: summary([r[m] for r in records if r["arm"] == "original"])
                     for m in MEASURES},
        "final": {m: summary([r[m] for r in records if r["arm"] == "final"])
                  for m in MEASURES},
        "paired_final_minus_original": {
            m: summary([p["final_minus_original"][m] for p in pairs]) for m in MEASURES
        },
        "pairs": pairs,
        "runs": records,
    }
    (RUN_DIR / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
    fields = ["label", "arm", "pair", "csv", "samples", *MEASURES,
              "command_limit_fraction", "start_angle_rad", "end_angle_rad"]
    with (RUN_DIR / "run_metrics.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(records)

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True, constrained_layout=True)
    colors = {"original": "#3565a4", "final": "#e66100"}
    representative = records[:2]
    for row in representative:
        data = data_by_label[row["label"]]
        center = np.median(data["angle"][:100])
        arm = row["arm"]
        axes[0].plot(data["t"], np.rad2deg(data["angle"] - center),
                     label=arm, color=colors[arm], linewidth=1)
        axes[1].plot(data["t"], np.rad2deg(data["error"]),
                     label=arm, color=colors[arm], linewidth=0.9)
        axes[2].plot(data["t"], data["measured_torque"],
                     label=arm, color=colors[arm], linewidth=0.8)
    axes[0].plot(common_time, np.rad2deg(reference), "k--", linewidth=1, label="target")
    axes[0].set_ylabel("relative yaw (deg)")
    axes[1].set_ylabel("control angle error (deg)")
    axes[2].set_ylabel("measured torque (N m)")
    axes[2].set_xlabel("time (s)")
    for ax in axes:
        ax.grid(alpha=0.3)
    axes[0].legend()
    fig.savefig(RUN_DIR / "representative_pair.png", dpi=160)
    plt.close(fig)
    for measure in MEASURES:
        print(measure, "original", result["original"][measure]["mean"],
              "final", result["final"][measure]["mean"],
              "paired delta", result["paired_final_minus_original"][measure]["mean"])
    print("maximum reference difference (rad)", max_reference_difference)


if __name__ == "__main__":
    main()
