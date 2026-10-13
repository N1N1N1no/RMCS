#!/usr/bin/env python3
"""Summarize completed C-car yaw controller A/B runs with chassis disabled."""

from __future__ import annotations

import json
from pathlib import Path

from compare_feedforward import load, metrics


ROOT = Path(__file__).resolve().parent
GROUPS = {
    "feedforward_forward": "current_position_ff_ab_chassis_disabled_2026-10-13",
    "feedforward_reverse": "current_position_ff_ab_chassis_disabled_reverse_2026-10-13",
    "angle_kp_forward": "current_position_angle_kp_ab_2026-10-13",
    "angle_kp_reverse": "current_position_angle_kp_ab_reverse_2026-10-13",
    "speed_kp_forward": "current_position_speed_kp_ab_2026-10-13",
    "speed_kp_reverse_incomplete": "current_position_speed_kp_ab_reverse_2026-10-13",
}
FIELDS = (
    "moving_rms_error_deg", "mean_moving_iae_rad_s", "median_late_abs_error_deg",
    "speed_peak_abs_rad_s", "measured_torque_p99_abs_Nm",
    "command_torque_peak_abs_Nm",
)


def main() -> None:
    results = {}
    for group, folder in GROUPS.items():
        source = ROOT / folder
        protocol = json.loads((source / "protocol.json").read_text())
        if protocol["chassis_components"] != "all chassis controllers disabled; feedback remains available":
            raise RuntimeError(f"Chassis control is not disabled in {folder}")
        manifest = json.loads((source / "manifest.json").read_text())
        rows = []
        for item in manifest:
            values = metrics(load(source / f"{item['label']}.csv"))
            rows.append({"arm": item["arm"], "label": item["label"],
                         "start_yaw_rad": item["start_angle_rad"],
                         **{key: values[key] for key in FIELDS}})
        results[group] = {"expected_order": protocol["order"], "rows": rows,
                          "complete": len(rows) == len(protocol["order"])}
    comparisons = {}
    for group, data in results.items():
        if len(data["rows"]) != 2:
            continue
        a, b = data["rows"]
        labels = {a["arm"], b["arm"]}
        if group.startswith("feedforward"):
            control, candidate = "baseline", "newff"
        elif group.startswith("angle_kp"):
            control, candidate = "newff", "angle14"
        elif group.startswith("speed_kp"):
            control, candidate = "newff", "speed13"
        else:
            continue
        if labels != {control, candidate}:
            raise RuntimeError(f"Unexpected paired arms: {group}")
        first = a if a["arm"] == control else b
        second = a if a["arm"] == candidate else b
        comparisons[group] = {
            "control": control, "candidate": candidate,
            "start_yaw_difference_rad": second["start_yaw_rad"] - first["start_yaw_rad"],
            "candidate_minus_control": {
                field: second[field] - first[field] for field in FIELDS
            },
            "candidate_relative_change": {
                field: second[field] / first[field] - 1 for field in FIELDS
            },
        }
    output = ROOT / "current_position_yaw_controller_ab_2026-10-13_analysis.json"
    output.write_text(json.dumps({"groups": results, "paired_comparisons": comparisons},
                                 ensure_ascii=False, indent=2) + "\n")
    for group, data in results.items():
        print(group, [(x["arm"], round(x["moving_rms_error_deg"], 5),
                       round(x["mean_moving_iae_rad_s"] * 1000, 5))
                      for x in data["rows"]], "complete", data["complete"])
    for group, data in comparisons.items():
        print(group, "candidate RMS change",
              round(100 * data["candidate_relative_change"]["moving_rms_error_deg"], 2),
              "%", "IAE change",
              round(100 * data["candidate_relative_change"]["mean_moving_iae_rad_s"], 2), "%")


if __name__ == "__main__":
    main()
