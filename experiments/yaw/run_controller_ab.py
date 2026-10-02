#!/usr/bin/env python3
"""Run three gated A/B pairs on the same smooth C-car yaw trajectory.

Run inside the RMCS devcontainer only after the operator confirms the test area.
Only the deployed C-car YAML is temporarily changed; the selected final YAML is
restored after completion or failure.
"""

import hashlib
import json
import time

import run_repeat_optimization as runner


runner.RUN_DIR = runner.ROOT / "controller_ab_2026-10-02"
ORDER = ["original", "final"] * 3
REPLACEMENTS = {
    "yaw_angle_kp: 12.0": "yaw_angle_kp: 10.0",
    "yaw_velocity_kp: 11.0": "yaw_velocity_kp: 13.0",
    "yaw_sweep_feedforward_scale: 1.0": "yaw_sweep_feedforward_scale: 0.0",
}


def original_config(final):
    original = final
    for old, new in REPLACEMENTS.items():
        if original.count(old) != 1:
            raise ValueError(f"Expected exactly one {old!r} in final YAML")
        original = original.replace(old, new)
    for expected in (
        "yaw_sweep_amplitude_rad: 0.10",
        "yaw_sweep_hold_seconds: 1.5",
        "yaw_sweep_cycles: 4",
        "yaw_sweep_transition_seconds: 0.5",
    ):
        if original.count(expected) != 1:
            raise ValueError(f"Unexpected smooth trajectory setting: {expected}")
    return original


def main():
    final = runner.SOURCE_YAML.read_text()
    original = original_config(final)
    if runner.ssh("hostname") != "alliance-hero":
        raise RuntimeError("Unexpected SSH target")
    local_library_hash = hashlib.sha256(runner.LIBRARY.read_bytes()).hexdigest()
    remote_library_hash = runner.ssh(f"sha256sum {runner.REMOTE_LIBRARY}").split()[0]
    if local_library_hash != remote_library_hash:
        raise RuntimeError("Robot library differs from compiled local library")
    if runner.ssh(f"cat {runner.REMOTE_YAML}") + "\n" != final:
        raise RuntimeError("Robot YAML differs from local selected final YAML")

    runner.RUN_DIR.mkdir(exist_ok=True)
    manifest_path = runner.RUN_DIR / "manifest.json"
    if manifest_path.exists() or list(runner.RUN_DIR.glob("*.csv")):
        raise RuntimeError("Output directory already contains A/B data")
    (runner.RUN_DIR / "protocol.json").write_text(json.dumps({
        "order": ORDER,
        "reference": "same 0.5 s quintic transitions, +/-0.10 rad, 1.5 s phases, four cycles",
        "original": {"yaw_angle_kp": 10.0, "yaw_velocity_kp": 13.0,
                     "yaw_sweep_feedforward_scale": 0.0},
        "final": {"yaw_angle_kp": 12.0, "yaw_velocity_kp": 11.0,
                  "yaw_sweep_feedforward_scale": 1.0,
                  "yaw_velocity_ff_gain": 1.378547846,
                  "yaw_acceleration_ff_gain": 0.142611646},
        "source_final_yaml_sha256": hashlib.sha256(final.encode()).hexdigest(),
        "original_yaml_sha256": hashlib.sha256(original.encode()).hexdigest(),
        "test_library_sha256": local_library_hash,
    }, indent=2) + "\n")

    configs = {"original": original, "final": final}
    manifest = []
    in_motion = False
    try:
        for number, arm in enumerate(ORDER, start=1):
            label = f"{number:02d}_{arm}"
            print(f"Starting {label}", flush=True)
            runner.deploy_config(configs[arm], f"yaw-controller-ab-{label}.yaml")
            logger = runner.latest_logger()
            runner.check_gate(logger)
            runner.trigger()
            in_motion = True
            time.sleep(16.5)
            result = runner.export_completed(logger, label)
            in_motion = False
            manifest.append({"label": label, "arm": arm, "pair": (number + 1) // 2,
                             "config_sha256": hashlib.sha256(configs[arm].encode()).hexdigest(),
                             "robot_logger": logger, **result})
            manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
            print(f"Finished {label}: {result}", flush=True)
    finally:
        if in_motion:
            try:
                runner.ssh("bash -lc 'source /root/env_setup.bash && "
                           "ros2 service call /rmcs/service/yaw_sweep/stop "
                           "std_srvs/srv/Trigger \"{}\"'", timeout=15)
                time.sleep(1)
            except Exception as exc:
                print(f"Could not stop active sweep: {exc}", flush=True)
        print("Restoring selected final C-car YAML", flush=True)
        runner.deploy_config(final, "yaw-controller-ab-restore-final.yaml")


if __name__ == "__main__":
    main()
