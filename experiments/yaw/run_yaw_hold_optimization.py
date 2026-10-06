#!/usr/bin/env python3
"""Compare Ki=0.05 and 0.08 on safe smooth yaw holds, restoring C-car YAML.

Run in the RMCS devcontainer only after the operator confirms safe robot motion.
"""

import hashlib
import json
import time
from datetime import datetime, timezone

import run_controller_ab_current as known
import run_repeat_optimization as remote


RUN_DIR = remote.ROOT / "smooth_ki_optimization_2026-10-02"
ORDER = [("smooth", arm) for arm in ("current", "ki_008", "ki_008", "current")]


def make_configs(current):
    candidate = known.replace_once(current, "yaw_velocity_ki: 0.05", "yaw_velocity_ki: 0.08")
    return {("smooth", "current"): current,
            ("smooth", "ki_008"): candidate}


def main():
    current = known.configs()["final"]
    configs = make_configs(current)
    if remote.ssh("hostname") != "alliance-hero":
        raise RuntimeError("Unexpected SSH target")
    if remote.ssh(f"sha256sum {remote.REMOTE_LIBRARY}").split()[0] != known.EXPECTED_LIBRARY_SHA256:
        raise RuntimeError("Unexpected robot library")
    deployed = remote.ssh(f"cat {remote.REMOTE_YAML}") + "\n"
    if deployed != current:
        raise RuntimeError("Robot C-car YAML differs from known Ki=0.05 baseline")
    if RUN_DIR.exists() and list(RUN_DIR.iterdir()):
        raise RuntimeError("Output directory is not empty")
    RUN_DIR.mkdir(exist_ok=True)
    remote.RUN_DIR = RUN_DIR
    (RUN_DIR / "protocol.json").write_text(json.dumps({
        "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
        "order": [{"shape": shape, "arm": arm} for shape, arm in ORDER],
        "reference": "four cycles, +/-0.10 rad, 1.5 s phases, 0.5 s quintic transitions",
        "current_velocity_ki": 0.05,
        "candidate_velocity_ki": 0.08,
        "other_gains_unchanged": True,
        "base_yaml_sha256": hashlib.sha256(current.encode()).hexdigest(),
        "library_sha256": known.EXPECTED_LIBRARY_SHA256,
        "config_sha256": {f"{shape}_{arm}": hashlib.sha256(config.encode()).hexdigest()
                          for (shape, arm), config in configs.items()},
    }, indent=2) + "\n")
    manifest = []
    in_motion = False
    try:
        for number, (shape, arm) in enumerate(ORDER, 1):
            label = f"{number:02d}_{shape}_{arm}"
            print(f"Starting {label}", flush=True)
            config = configs[(shape, arm)]
            remote.deploy_config(config, f"yaw-hold-ki-{label}.yaml")
            logger = remote.latest_logger()
            remote.check_gate(logger)
            remote.trigger()
            in_motion = True
            time.sleep(16.5)
            result = remote.export_completed(logger, label)
            in_motion = False
            manifest.append({"label": label, "shape": shape, "arm": arm,
                             "pair": (number - 1) % 4 // 2 + 1,
                             "config_sha256": hashlib.sha256(config.encode()).hexdigest(),
                             "robot_logger": logger, **result})
            (RUN_DIR / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
            print(f"Finished {label}: {result}", flush=True)
            if result["max_command_torque_Nm"] > 4.05 or result["max_speed_rad_s"] > 1.0:
                raise RuntimeError(f"Stopping after approaching a motion limit: {label}")
    finally:
        if in_motion:
            try:
                remote.ssh("bash -lc 'source /root/env_setup.bash && "
                           "ros2 service call /rmcs/service/yaw_sweep/stop "
                           "std_srvs/srv/Trigger \"{}\"'", timeout=15)
                time.sleep(1)
            except Exception as exc:
                print(f"Could not stop active sweep: {exc}", flush=True)
        print("Restoring exact Ki=0.05 C-car YAML", flush=True)
        remote.deploy_config(deployed, "yaw-hold-ki-restore.yaml")
        if remote.ssh(f"sha256sum {remote.REMOTE_YAML}").split()[0] != known.EXPECTED_CURRENT_SHA256:
            raise RuntimeError("C-car YAML restoration failed")


if __name__ == "__main__":
    main()
