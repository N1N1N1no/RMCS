#!/usr/bin/env python3
"""Reproduce the 7.5/8.33 Hz yaw checks motivated by auto-aim plate changes."""

from __future__ import annotations

import json
from pathlib import Path

import analyze_frequency_yaw_retest as earlier


ROOT = Path(__file__).resolve().parent
DATA = ROOT / "frequency_yaw_retest_2026-10-13_autoaim_band"
OUTPUT = ROOT / "autoaim_band_2026-10-13_analysis.json"

MODELS = {
    "previous_yaw_about_2rad_0p5Nm": {
        "J_kg_m2": 0.1215279090,
        "B_Nm_s_per_rad": 4.7357662537,
    },
    "previous_yaw_about_0p2rad_0p5Nm": {
        "J_kg_m2": 0.11165369034168546,
        "B_Nm_s_per_rad": 2.5980686485313185,
    },
    "previous_yaw_about_5rad_0p5Nm": {
        "J_kg_m2": 0.0789667616,
        "B_Nm_s_per_rad": 2.7770818559,
    },
}


def main() -> None:
    manifest = json.loads((DATA / "manifest.json").read_text())
    records = [earlier.read_run(DATA, item) for item in manifest]
    if {item["label"] for item in records} != {
        "f7p5_run1", "f7p5_run2", "f8p333_run1", "f8p333_run2"
    }:
        raise RuntimeError("Expected exactly two repeats at both frequencies")
    if not all(item["completed"] and item["torque_command_Nm"] == 0.5
               for item in records):
        raise RuntimeError("Incomplete or changed-amplitude run")

    discovery = [item for item in records if item["label"].endswith("run1")]
    validation = [item for item in records if item["label"].endswith("run2")]
    highband_fit = earlier.fit(discovery)
    models = {"highband_fit_on_run1": highband_fit, **MODELS}
    result = {
        "source": str(DATA),
        "input": "measured motor-current-derived torque",
        "output": "relative IMU yaw speed; encoder angle cross-check",
        "excitation": "+ - - + four-phase torque, fundamental 1/(4*phase_seconds)",
        "quality_gate": {
            "speed_harmonic_to_residual_std_min": 3.0,
            "angle_harmonic_to_residual_std_min_for_angle_claim": 3.0,
        },
        "records": [earlier.public(item) for item in records],
        "start_yaw_span_rad": [min(item["start_angle_rad"] for item in records),
                               max(item["start_angle_rad"] for item in records)],
        "exploratory_highband_fit": highband_fit,
        "run2_fixed_model_scores": {
            name: earlier.score(validation, parameters)
            for name, parameters in models.items()
        },
        "interpretation": [
            "Both frequencies pass the relative-speed harmonic quality gate.",
            "All four encoder-angle harmonics fail the angle quality gate.",
            "Frequency groups begin at different yaw angles; the highband fit is not a position-general model.",
            "No actual auto-aim direction or target-following error was recorded.",
        ],
    }
    OUTPUT.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print("exploratory fit", highband_fit)
    for name, score in result["run2_fixed_model_scores"].items():
        print(name, score["mean_relative_torque_phasor_error_from_imu_speed"],
              score["mean_relative_torque_phasor_error_from_encoder_angle"])
    print("wrote", OUTPUT)


if __name__ == "__main__":
    main()
