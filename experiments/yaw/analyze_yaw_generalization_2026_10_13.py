#!/usr/bin/env python3
"""Predefined whole-run holdout comparison of local and pooled C-car yaw models.

Only 0.5 N m, completed runs with usable relative-IMU velocity enter parameter
selection. Second runs and separate collection sessions remain untouched until
scoring. Encoder-angle scores are reported only when its harmonic agrees with
the independent relative-IMU velocity harmonic.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

import analyze_frequency_yaw_retest as sweep


ROOT = Path(__file__).resolve().parent
DIRS = {
    "yaw_5p3": ("frequency_yaw_retest_2026-10-13_low",
                 "frequency_yaw_retest_2026-10-13_pilot_attempt2",
                 "frequency_yaw_retest_2026-10-13_high"),
    "yaw_2p0": ("frequency_yaw_retest_2026-10-13_cross_position",),
    "yaw_2p5": ("frequency_yaw_retest_2026-10-13_matched_generalization",),
}
TEST_DIRS = {
    "yaw_5p3_repeat": DIRS["yaw_5p3"],
    "yaw_4p8_move": ("frequency_yaw_retest_2026-10-13_validation",),
    "yaw_2p0_repeat": ("frequency_yaw_retest_2026-10-13_cross_position_repeat",),
    "yaw_2p5_repeat": DIRS["yaw_2p5"],
    "yaw_2p0_at_0p7Nm": ("frequency_yaw_retest_2026-10-13_cross_position_0p7nm_high",),
    "yaw_0p1_at_7p5_8p3Hz": ("frequency_yaw_retest_2026-10-13_autoaim_band",),
}


def load(names: tuple[str, ...]) -> list[dict]:
    return sweep.load_group(tuple(ROOT / name for name in names))


def good_speed(r: dict) -> bool:
    return (r["completed"] and r["speed_harmonic_to_residual_std"] >= 3.0
            and r["torque_command_Nm"] == 0.5)


def angle_check(r: dict) -> dict:
    derivative = 1j * 2 * np.pi * r["frequency_hz"] * r["angle_phasor"]
    ratio = derivative / r["speed_phasor"]
    gain = float(abs(ratio))
    phase = float(np.angle(ratio, deg=True))
    return {
        "encoder_derivative_to_imu_speed_gain": gain,
        "encoder_derivative_to_imu_speed_phase_deg": phase,
        "angle_harmonic_to_residual_std": r["angle_harmonic_to_residual_std"],
        "angle_quality_pass": (r["angle_harmonic_to_residual_std"] >= 3.0
                               and abs(gain - 1.0) <= 0.25 and abs(phase) <= 15.0),
    }


def fit_shared_j(groups: dict[str, list[dict]]) -> dict:
    names = tuple(groups)
    rows, values = [], []
    for group_index, group_name in enumerate(names):
        for r in groups[group_name]:
            velocity = r["speed_phasor"] / abs(r["torque_phasor"])
            target = r["torque_phasor"] / abs(r["torque_phasor"])
            row = np.zeros(len(names) + 1, dtype=complex)
            row[group_index] = velocity
            row[-1] = 1j * 2 * np.pi * r["frequency_hz"] * velocity
            rows.extend((row.real, row.imag))
            values.extend((target.real, target.imag))
    coefficients = np.linalg.lstsq(np.array(rows), np.array(values), rcond=None)[0]
    return {"J_kg_m2": float(coefficients[-1]),
            "B_by_group_Nm_s_per_rad": dict(zip(names, map(float, coefficients[:-1])))}


def score_group(records: list[dict], parameters: dict) -> dict:
    rows = []
    for r in records:
        errors = sweep.error(r, parameters)
        quality = angle_check(r)
        rows.append({
            "label": r["label"], "frequency_hz": r["frequency_hz"],
            "start_angle_rad": r["start_angle_rad"],
            "speed_quality": r["speed_harmonic_to_residual_std"],
            "relative_torque_error_from_imu_speed":
                errors["relative_torque_phasor_error_from_imu_speed"],
            "relative_torque_error_from_encoder_angle_if_quality_pass":
                (errors["relative_torque_phasor_error_from_encoder_angle"]
                 if quality["angle_quality_pass"] else None),
            **quality,
        })
    encoder_errors = [row["relative_torque_error_from_encoder_angle_if_quality_pass"]
                      for row in rows if row["angle_quality_pass"]]
    return {
        "run_count": len(rows),
        "mean_relative_torque_error_from_imu_speed": float(np.mean([
            row["relative_torque_error_from_imu_speed"] for row in rows])),
        "angle_qualified_run_count": len(encoder_errors),
        "mean_relative_torque_error_from_encoder_angle_if_quality_pass":
            float(np.mean(encoder_errors)) if encoder_errors else None,
        "runs": rows,
    }


def main() -> None:
    train = {}
    for name, directories in DIRS.items():
        candidates = load(directories)
        train[name] = [r for r in candidates if good_speed(r)
                       and r["label"].endswith("run1")
                       and 1.25 <= r["frequency_hz"] <= 6.25]
        if len(train[name]) < 2:
            raise RuntimeError(f"Too few training frequencies for {name}")
    local = {name: sweep.fit(records) for name, records in train.items()}
    pooled = sweep.fit([r for records in train.values() for r in records])
    shared = fit_shared_j(train)
    matching = {
        "yaw_5p3_repeat": "yaw_5p3", "yaw_4p8_move": "yaw_5p3",
        "yaw_2p0_repeat": "yaw_2p0", "yaw_2p5_repeat": "yaw_2p5",
        "yaw_2p0_at_0p7Nm": "yaw_2p0",
        "yaw_0p1_at_7p5_8p3Hz": "yaw_5p3",
    }
    validation = {}
    for name, directories in TEST_DIRS.items():
        candidates = load(directories)
        records = [r for r in candidates if r["completed"]
                   and r["speed_harmonic_to_residual_std"] >= 3.0]
        if name in ("yaw_5p3_repeat", "yaw_2p5_repeat", "yaw_0p1_at_7p5_8p3Hz"):
            records = [r for r in records if r["label"].endswith("run2")]
        if name == "yaw_5p3_repeat":
            records = [r for r in records if 1.25 <= r["frequency_hz"] <= 6.25]
        if name == "yaw_4p8_move":
            records = [r for r in records if 2.083 <= r["frequency_hz"] <= 6.25]
        if not records:
            raise RuntimeError(f"No complete holdout records: {name}")
        group = matching[name]
        models = {
            "local": local[group], "pooled": pooled,
            "shared_J_local_B": {
                "J_kg_m2": shared["J_kg_m2"],
                "B_Nm_s_per_rad": shared["B_by_group_Nm_s_per_rad"][group],
            },
        }
        validation[name] = {
            "assigned_training_group": group,
            "models": {key: score_group(records, parameters)
                       for key, parameters in models.items()},
        }
    primary = ("yaw_5p3_repeat", "yaw_4p8_move", "yaw_2p0_repeat",
               "yaw_2p5_repeat")
    summary = {}
    for model in ("local", "pooled", "shared_J_local_B"):
        group_means = [validation[name]["models"][model][
            "mean_relative_torque_error_from_imu_speed"] for name in primary]
        summary[model] = {
            "primary_holdout_group_mean_error": float(np.mean(group_means)),
            "primary_holdout_worst_group_mean_error": float(np.max(group_means)),
            "per_group_mean_error": dict(zip(primary, group_means)),
        }
    high_amplitude = load(("frequency_yaw_retest_2026-10-13_matched_0p7nm",))
    high_train = [r for r in high_amplitude if r["completed"]
                  and r["label"].endswith("run1")]
    high_test = [r for r in high_amplitude if r["completed"]
                 and r["label"].endswith("run2")]
    if len(high_train) != 2 or len(high_test) != 2:
        raise RuntimeError("Expected two complete 0.7 N m frequencies and repeats")
    high_fit = sweep.fit(high_train)
    historical_high = {"J_kg_m2": 0.09285235610400416,
                       "B_Nm_s_per_rad": 1.349998652800123}
    secondary = {
        "train_amplitude_Nm": 0.7,
        "position_note": "6.21-6.24 rad start; not matched with 0.5 N m at 2.50-2.55 rad",
        "train": [{"label": r["label"], "frequency_hz": r["frequency_hz"],
                   "start_angle_rad": r["start_angle_rad"], "sha256": r["sha256"]}
                  for r in high_train],
        "new_local_fit": high_fit,
        "historical_0p7Nm_0p11rad_fit": historical_high,
        "holdout_new_local": score_group(high_test, high_fit),
        "holdout_historical": score_group(high_test, historical_high),
    }
    result = {
        "measurement": "measured torque / relative IMU yaw speed fundamental phasor",
        "model": "torque = (B+j*2*pi*f*J)*relative_yaw_speed",
        "quality_rules": {
            "complete_runs_only": True,
            "train_amplitude_Nm": 0.5,
            "minimum_speed_harmonic_to_residual_std": 3.0,
            "angle_minimum_harmonic_to_residual_std": 3.0,
            "angle_to_imu_gain_tolerance": 0.25,
            "angle_to_imu_phase_tolerance_deg": 15.0,
        },
        "train": {name: [{"label": r["label"], "frequency_hz": r["frequency_hz"],
                          "start_angle_rad": r["start_angle_rad"], "sha256": r["sha256"]}
                         for r in records] for name, records in train.items()},
        "fits": {"local": local, "pooled": pooled, "shared_J_local_B": shared},
        "primary_holdout_summary": summary,
        "validation": validation,
        "secondary_0p7Nm_6p2rad": secondary,
        "limitations": [
            "The new 2.5 rad position was tested under 0.5 N m only.",
            "A speed-harmonic fit does not prove closed-loop auto-aim tracking.",
            "The 6.25 Hz encoder angle at the new position fails angle quality.",
            "The 0.7 N m and 7.5-8.333 Hz sets are stress tests, not model-selection data.",
        ],
    }
    output = ROOT / "yaw_generalization_2026-10-13_analysis.json"
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print("Wrote", output)
    print("fits:", json.dumps(result["fits"], ensure_ascii=False))
    print("primary holdout:", json.dumps(summary, ensure_ascii=False))
    for name, item in validation.items():
        print(name, {key: round(score["mean_relative_torque_error_from_imu_speed"], 3)
                     for key, score in item["models"].items()})
    print("secondary 0.7 N m:", high_fit,
          "new holdout", round(secondary["holdout_new_local"][
              "mean_relative_torque_error_from_imu_speed"], 3),
          "historical holdout", round(secondary["holdout_historical"][
              "mean_relative_torque_error_from_imu_speed"], 3))


if __name__ == "__main__":
    main()
