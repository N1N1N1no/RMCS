#!/usr/bin/env python3
"""Interleaved C-car yaw velocity-integral A/B on a slower smooth trajectory.

Run in the RMCS development container after the physical test area is cleared.
Only C-car YAML is temporarily deployed; the exact prior YAML is restored.
"""

import argparse
import hashlib
import json
import subprocess
import time

import run_repeat_optimization as remote


ROOT = remote.ROOT
RUN_DIR = ROOT / "low_speed_ki_ab_2026-10-02"
ORDER = ["current", "ki_005"] * 3
YAML = "rmcs_ws/src/rmcs_bringup/config/deformable-infantry-omni-c.yaml"


def variant(base, ki, *, slow=True):
    changes = {"yaw_velocity_ki: 0.02": f"yaw_velocity_ki: {ki:.2f}"}
    if slow:
        changes.update({
            "yaw_sweep_amplitude_rad: 0.10": "yaw_sweep_amplitude_rad: 0.08",
            "yaw_sweep_transition_seconds: 0.5": "yaw_sweep_transition_seconds: 1.0",
        })
    for old, new in changes.items():
        if base.count(old) != 1:
            raise ValueError(f"Unexpected C-car YAML: {old}")
        base = base.replace(old, new)
    return base


def main():
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--reverse", action="store_true")
    group.add_argument("--fast", action="store_true")
    args = parser.parse_args()
    global RUN_DIR, ORDER
    if args.reverse:
        ORDER = ["ki_005", "current"] * 3
        RUN_DIR = ROOT / "low_speed_ki_ab_reverse_2026-10-02"
    elif args.fast:
        ORDER = ["current", "ki_005", "ki_005", "current"]
        RUN_DIR = ROOT / "fast_speed_ki_ab_2026-10-02"
    if remote.ssh("hostname") != "alliance-hero":
        raise RuntimeError("Unexpected SSH target")
    expected = subprocess.check_output(["git", "show", f"HEAD:{YAML}"]).decode()
    deployed = remote.ssh(f"cat {remote.REMOTE_YAML}") + "\n"
    if deployed != expected:
        raise RuntimeError("Robot C-car YAML differs from the known pre-test version")
    old_library_sha = remote.ssh(f"sha256sum {remote.REMOTE_LIBRARY}").split()[0]
    if old_library_sha != "2a046d42b9b5b389691688ffe89263a221e82579e9ac425ccf015f51d7431b3c":
        raise RuntimeError("Robot library is not the expected pre-test build")
    if RUN_DIR.exists() and list(RUN_DIR.iterdir()):
        raise RuntimeError("Output directory already contains experiment files")
    RUN_DIR.mkdir(exist_ok=True)
    remote.RUN_DIR = RUN_DIR
    configs = {"current": variant(expected, 0.02, slow=not args.fast),
               "ki_005": variant(expected, 0.05, slow=not args.fast)}
    (RUN_DIR / "protocol.json").write_text(json.dumps({
        "order": ORDER,
        "trajectory": {"amplitude_rad": 0.10 if args.fast else 0.08,
                       "transition_s": 0.5 if args.fast else 1.0,
                       "hold_s": 1.5, "cycles": 4},
        "current_ki": 0.02, "candidate_ki": 0.05,
        "original_yaml_sha256": hashlib.sha256(expected.encode()).hexdigest(),
        "library_sha256": old_library_sha,
        "config_sha256": {arm: hashlib.sha256(cfg.encode()).hexdigest()
                          for arm, cfg in configs.items()},
    }, indent=2) + "\n")
    manifest = []
    in_motion = False
    try:
        for number, arm in enumerate(ORDER, 1):
            label = f"{number:02d}_{arm}"
            print(f"Starting {label}", flush=True)
            remote.deploy_config(configs[arm], f"yaw-low-speed-ki-{label}.yaml")
            logger = remote.latest_logger()
            remote.check_gate(logger)
            remote.trigger()
            in_motion = True
            time.sleep(16.5)
            result = remote.export_completed(logger, label)
            in_motion = False
            manifest.append({"label": label, "arm": arm,
                             "pair": (number + 1) // 2,
                             "config_sha256": hashlib.sha256(configs[arm].encode()).hexdigest(),
                             "robot_logger": logger, **result})
            (RUN_DIR / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
            print(f"Finished {label}: {result}", flush=True)
            if (result["max_speed_rad_s"] > 1.0
                or result["max_command_torque_Nm"] > 3.0
                or result["max_error_deg"] > 3.0):
                raise RuntimeError(f"Stopping A/B due to little motion headroom: {label}")
    finally:
        if in_motion:
            try:
                remote.ssh("bash -lc 'source /root/env_setup.bash && "
                           "ros2 service call /rmcs/service/yaw_sweep/stop "
                           "std_srvs/srv/Trigger \"{}\"'", timeout=15)
                time.sleep(1)
            except Exception as exc:
                print(f"Could not stop active sweep: {exc}", flush=True)
        print("Restoring original C-car YAML", flush=True)
        remote.deploy_config(expected, "yaw-low-speed-ki-restore.yaml")
        if remote.ssh(f"sha256sum {remote.REMOTE_YAML}").split()[0] != hashlib.sha256(expected.encode()).hexdigest():
            raise RuntimeError("Robot YAML restoration failed")


if __name__ == "__main__":
    main()
