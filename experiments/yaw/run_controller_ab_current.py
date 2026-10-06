#!/usr/bin/env python3
"""Repeat the original-versus-current C-car yaw A/B on the original smooth path.

Run in the RMCS devcontainer after the operator confirms the robot is secured.
The original arm uses the historical Kp 10/13, Ki 0.02 and no feedforward;
the current arm uses Kp 12/11, Ki 0.05 and the identified feedforward.
The robot's pre-test YAML is restored even if a trial fails.
"""

import hashlib
import json
import subprocess
import time
from datetime import datetime, timezone

import run_repeat_optimization as remote


ROOT = remote.ROOT.parents[1]
RUN_DIR = remote.ROOT / "controller_ab_current_2026-10-02"
YAML_PATH = "rmcs_ws/src/rmcs_bringup/config/deformable-infantry-omni-c.yaml"
EXPECTED_LIBRARY_SHA256 = "2a046d42b9b5b389691688ffe89263a221e82579e9ac425ccf015f51d7431b3c"
EXPECTED_CURRENT_SHA256 = "f2e4048cd65ecf90c74828538e502616f62b0e3e0d3d558a35f99d9ed5ceca18"
ORDER = ["original", "final"] * 3


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError(f"Expected exactly one {old!r} in the reference C-car YAML")
    return text.replace(old, new)


def configs():
    historical = subprocess.check_output(
        ["git", "show", f"HEAD:{YAML_PATH}"], cwd=ROOT
    ).decode()
    original = replace_once(historical, "yaw_angle_kp: 12.0", "yaw_angle_kp: 10.0")
    original = replace_once(original, "yaw_velocity_kp: 11.0", "yaw_velocity_kp: 13.0")
    original = replace_once(
        original, "yaw_sweep_feedforward_scale: 1.0", "yaw_sweep_feedforward_scale: 0.0"
    )
    final = replace_once(historical, "yaw_velocity_ki: 0.02", "yaw_velocity_ki: 0.05")
    if hashlib.sha256(final.encode()).hexdigest() != EXPECTED_CURRENT_SHA256:
        raise ValueError("Historical base does not reproduce the deployed current YAML")
    for setting in (
        "yaw_sweep_amplitude_rad: 0.10",
        "yaw_sweep_hold_seconds: 1.5",
        "yaw_sweep_cycles: 4",
        "yaw_sweep_transition_seconds: 0.5",
    ):
        if final.count(setting) != 1 or original.count(setting) != 1:
            raise ValueError(f"Unexpected trajectory setting {setting}")
    return {"original": original, "final": final}


def main():
    arms = configs()
    if remote.ssh("hostname") != "alliance-hero":
        raise RuntimeError("Unexpected SSH target")
    if remote.ssh(f"sha256sum {remote.REMOTE_LIBRARY}").split()[0] != EXPECTED_LIBRARY_SHA256:
        raise RuntimeError("Robot library differs from the known tested build")
    deployed = remote.ssh(f"cat {remote.REMOTE_YAML}") + "\n"
    if deployed != arms["final"]:
        raise RuntimeError("Robot C-car YAML differs from the known Ki=0.05 configuration")
    if RUN_DIR.exists() and list(RUN_DIR.iterdir()):
        raise RuntimeError("New A/B output directory is not empty")
    RUN_DIR.mkdir(exist_ok=True)
    remote.RUN_DIR = RUN_DIR
    (RUN_DIR / "protocol.json").write_text(json.dumps({
        "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
        "order": ORDER,
        "reference": "same 0.5 s quintic transitions, +/-0.10 rad, 1.5 s phases, four cycles",
        "original": {"yaw_angle_kp": 10.0, "yaw_velocity_kp": 13.0,
                     "yaw_velocity_ki": 0.02, "yaw_sweep_feedforward_scale": 0.0},
        "final": {"yaw_angle_kp": 12.0, "yaw_velocity_kp": 11.0,
                  "yaw_velocity_ki": 0.05, "yaw_sweep_feedforward_scale": 1.0,
                  "yaw_velocity_ff_gain": 1.378547846,
                  "yaw_acceleration_ff_gain": 0.142611646},
        "source_final_yaml_sha256": hashlib.sha256(arms["final"].encode()).hexdigest(),
        "original_yaml_sha256": hashlib.sha256(arms["original"].encode()).hexdigest(),
        "test_library_sha256": EXPECTED_LIBRARY_SHA256,
    }, indent=2) + "\n")
    manifest = []
    in_motion = False
    try:
        for number, arm in enumerate(ORDER, start=1):
            label = f"{number:02d}_{arm}"
            print(f"Starting {label}", flush=True)
            remote.deploy_config(arms[arm], f"yaw-controller-ab-current-{label}.yaml")
            logger = remote.latest_logger()
            remote.check_gate(logger)
            remote.trigger()
            in_motion = True
            time.sleep(16.5)
            result = remote.export_completed(logger, label)
            in_motion = False
            manifest.append({"label": label, "arm": arm, "pair": (number + 1) // 2,
                             "config_sha256": hashlib.sha256(arms[arm].encode()).hexdigest(),
                             "robot_logger": logger, **result})
            (RUN_DIR / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
            print(f"Finished {label}: {result}", flush=True)
            if result["max_error_deg"] > 5.0:
                raise RuntimeError(f"Stopping A/B after excessive tracking error: {label}")
    finally:
        if in_motion:
            try:
                remote.ssh("bash -lc 'source /root/env_setup.bash && "
                           "ros2 service call /rmcs/service/yaw_sweep/stop "
                           "std_srvs/srv/Trigger \"{}\"'", timeout=15)
                time.sleep(1)
            except Exception as exc:
                print(f"Could not stop active sweep: {exc}", flush=True)
        print("Restoring exact pre-test C-car YAML", flush=True)
        remote.deploy_config(deployed, "yaw-controller-ab-current-restore.yaml")
        if remote.ssh(f"sha256sum {remote.REMOTE_YAML}").split()[0] != EXPECTED_CURRENT_SHA256:
            raise RuntimeError("C-car YAML restoration failed")


if __name__ == "__main__":
    main()
