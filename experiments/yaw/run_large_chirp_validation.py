#!/usr/bin/env python3
"""Run gated, increasing-amplitude C-car torque chirps and export every CSV.

Execute in the RMCS development container after the operator has cleared the
robot. The previously deployed library and YAML are restored even on failure.
"""

import argparse
import csv
import hashlib
import json
import math
import subprocess
import time
from pathlib import Path

import run_repeat_optimization as remote


ROOT = Path(__file__).resolve().parent
RUN_DIR = ROOT / "large_chirp_2026-10-02"
CPP = "rmcs_ws/src/rmcs_core/src/hardware/deformable-infantry-omni-c.cpp"
YAML = "rmcs_ws/src/rmcs_bringup/config/deformable-infantry-omni-c.yaml"
PLAN = [
    ("01_low_005", 0.05, 0.4, 1.1),
    ("02_low_010", 0.10, 0.4, 1.1),
    ("03_low_015", 0.15, 0.4, 1.1),
    ("04_low_015_repeat", 0.15, 0.4, 1.1),
    ("05_high_006", 0.06, 1.0, 2.5),
]
FOLLOWUP_PLAN = [
    ("01_high_006", 0.06, 1.0, 2.5),
    ("02_low_012", 0.12, 0.4, 1.1),
    ("03_low_013", 0.13, 0.4, 1.1),
    ("04_low_014", 0.14, 0.4, 1.1),
]
EXTENDED_HIGH_PLAN = [
    ("01_high_003", 0.03, 2.5, 3.0),
    ("02_high_004", 0.04, 2.5, 3.0),
]
EXTENDED_LOW_PLAN = [
    ("01_verylow_008", 0.08, 0.3, 0.65),
    ("02_verylow_011", 0.11, 0.3, 0.65),
]
BRIDGE_PLAN = [
    ("01_midlow_008", 0.08, 0.4, 1.1),
    ("02_midhigh_004", 0.04, 1.0, 2.5),
]


def sha(data):
    return hashlib.sha256(data).hexdigest()


def variant(base, amplitude, start, end):
    result = base
    for old, new in (
        ("yaw_chirp_amplitude_rad: 0.05", f"yaw_chirp_amplitude_rad: {amplitude:.2f}"),
        ("yaw_chirp_start_hz: 0.4", f"yaw_chirp_start_hz: {start:.1f}"),
        ("yaw_chirp_end_hz: 1.1", f"yaw_chirp_end_hz: {end:.2f}"),
    ):
        if result.count(old) != 1:
            raise ValueError(f"Unexpected source config: {old}")
        result = result.replace(old, new)
    return result


def service(name):
    response = remote.ssh(
        "bash -lc 'source /root/env_setup.bash && ros2 service call "
        f"/rmcs/service/yaw_chirp/{name} std_srvs/srv/Trigger \"{{}}\"'",
        timeout=15,
    )
    if "success=True" not in response:
        raise RuntimeError(f"Yaw chirp {name} refused: {response}")


def newest_chirp():
    return remote.ssh("ls -t /tmp/yaw_chirp_*.csv 2>/dev/null | head -1")


def export_chirp(robot_path, label):
    path = RUN_DIR / f"{label}.csv"
    remote.call("scp", "-q", f"remote:{robot_path}", str(path), timeout=30)
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError(f"Empty chirp CSV: {label}")
    def column(key):
        return [float(row[key]) for row in rows]
    time_s = column("t_s")
    initial = float(rows[0]["angle_rad"])
    displacement = [math.remainder(v - initial, 2 * math.pi) for v in column("angle_rad")]
    motor_speed = column("velocity_rad_s")
    torque = column("torque_command_nm")
    target = column("target_offset_rad")
    if not all(math.isfinite(v) for v in time_s + displacement + motor_speed + torque + target):
        raise RuntimeError(f"Nonfinite chirp telemetry: {label}")
    completed = len(rows) >= 7400 and time_s[-1] >= 7.9
    result = {
        "csv": str(path), "robot_csv": robot_path, "samples": len(rows),
        "duration_s": time_s[-1], "completed": completed,
        "max_abs_target_rad": max(map(abs, target)),
        "max_abs_displacement_rad": max(map(abs, displacement)),
        "max_abs_motor_speed_rad_s": max(map(abs, motor_speed)),
        "max_abs_command_torque_Nm": max(map(abs, torque)),
        "start_angle_rad": initial,
    }
    if result["max_abs_displacement_rad"] > 0.181 or result["max_abs_motor_speed_rad_s"] > 1.201:
        raise RuntimeError(f"Chirp exceeded motion boundary: {label}: {result}")
    if result["max_abs_command_torque_Nm"] > 3.001:
        raise RuntimeError(f"Chirp exceeded torque boundary: {label}: {result}")
    return result


def main():
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--followup", action="store_true")
    group.add_argument("--extended-high", action="store_true")
    group.add_argument("--extended-low", action="store_true")
    group.add_argument("--bridge", action="store_true")
    args = parser.parse_args()
    global PLAN, RUN_DIR
    if args.followup:
        PLAN = FOLLOWUP_PLAN
        RUN_DIR = ROOT / "large_chirp_followup_2026-10-02"
    elif args.extended_high:
        PLAN = EXTENDED_HIGH_PLAN
        RUN_DIR = ROOT / "chirp_extended_high_2026-10-02"
    elif args.extended_low:
        PLAN = EXTENDED_LOW_PLAN
        RUN_DIR = ROOT / "chirp_extended_low_2026-10-02"
    elif args.bridge:
        PLAN = BRIDGE_PLAN
        RUN_DIR = ROOT / "chirp_bridge_2026-10-02"
    if remote.ssh("hostname") != "alliance-hero":
        raise RuntimeError("Unexpected SSH target")
    old_yaml = subprocess.check_output(["git", "show", f"HEAD:{YAML}"]).decode()
    source_yaml = remote.SOURCE_YAML.read_text()
    if remote.ssh(f"cat {remote.REMOTE_YAML}") + "\n" != old_yaml:
        raise RuntimeError("Robot C-car YAML differs from the expected pre-test version")
    if "deformable-infantry-omni-c.yaml" not in remote.ssh("ps -eo args"):
        raise RuntimeError("C-car executor is not running")
    source_library = remote.LIBRARY.read_bytes()
    old_library_sha = remote.ssh(f"sha256sum {remote.REMOTE_LIBRARY}").split()[0]
    new_library_sha = sha(source_library)
    if old_library_sha == new_library_sha:
        raise RuntimeError("Compiled test library does not differ from deployed library")
    if RUN_DIR.exists() and list(RUN_DIR.iterdir()):
        raise RuntimeError("Experiment output directory already contains files")
    RUN_DIR.mkdir(exist_ok=True)
    (RUN_DIR / "protocol.json").write_text(json.dumps({
        "plan": PLAN, "model": {"J": 0.142611646, "B": 1.378547846, "delay_s": 0.003},
        "chirp_duration_s": 8.0, "torque_cap_Nm": 3.0,
        "displacement_stop_rad": 0.18, "speed_stop_rad_s": 1.2,
        "old_yaml_sha256": sha(old_yaml.encode()),
        "test_yaml_sha256": sha(source_yaml.encode()),
        "old_library_sha256": old_library_sha,
        "test_library_sha256": new_library_sha,
        "cpp_source_sha256": sha((ROOT.parents[1] / CPP).read_bytes()),
    }, indent=2) + "\n")

    stamp = str(int(time.time()))
    backup_lib = f"/tmp/librmcs_core_before_chirp_{stamp}.so"
    backup_yaml = f"/tmp/deformable_infantry_omni_c_before_chirp_{stamp}.yaml"
    remote.ssh(f"cp {remote.REMOTE_LIBRARY} {backup_lib} && "
               f"cp {remote.REMOTE_YAML} {backup_yaml}")
    remote.call("scp", "-q", str(remote.LIBRARY), "remote:/tmp/librmcs_core_chirp_test.so")
    manifest = []
    in_motion = False
    try:
        remote.ssh("set -e; service rmcs stop; "
                   f"install -m 644 /tmp/librmcs_core_chirp_test.so {remote.REMOTE_LIBRARY}; "
                   "service rmcs start", timeout=30)
        time.sleep(2)
        if remote.ssh(f"sha256sum {remote.REMOTE_LIBRARY}").split()[0] != new_library_sha:
            raise RuntimeError("Robot test library hash mismatch")
        for run_index, (label, amplitude, start_hz, end_hz) in enumerate(PLAN):
            print(f"Starting {label}: {amplitude:.2f} rad, {start_hz:.1f}-{end_hz:.1f} Hz", flush=True)
            config = variant(source_yaml, amplitude, start_hz, end_hz)
            remote.deploy_config(config, f"yaw-chirp-{label}.yaml")
            remote.check_gate(remote.latest_logger())
            previous = newest_chirp()
            in_motion = True
            service("start")
            time.sleep(8.7)
            robot_path = newest_chirp()
            if not robot_path or robot_path == previous:
                raise RuntimeError(f"No fresh chirp CSV for {label}; start may have been refused")
            result = export_chirp(robot_path, label)
            in_motion = False
            manifest.append({"label": label, "amplitude_rad": amplitude,
                             "start_hz": start_hz, "end_hz": end_hz,
                             "config_sha256": sha(config.encode()), **result})
            (RUN_DIR / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
            print(f"Finished {label}: {result}", flush=True)
            if not result["completed"]:
                raise RuntimeError(f"Chirp stopped early; no larger amplitude will be attempted: {label}")
            if run_index + 1 < len(PLAN) and (
                result["max_abs_displacement_rad"] > 0.16
                or result["max_abs_motor_speed_rad_s"] > 1.08
                or result["max_abs_command_torque_Nm"] > 2.65
            ):
                raise RuntimeError(
                    f"Not increasing amplitude: motion headroom is too small after {label}"
                )
    finally:
        if in_motion:
            try:
                service("stop")
                time.sleep(1)
            except Exception as exc:
                print(f"Could not stop chirp through service: {exc}", flush=True)
        print("Restoring previously deployed robot library and C-car YAML", flush=True)
        remote.ssh("set -e; service rmcs stop; "
                   f"install -m 644 {backup_lib} {remote.REMOTE_LIBRARY}; "
                   f"install -m 644 {backup_yaml} {remote.REMOTE_YAML}; "
                   "service rmcs start", timeout=30)
        time.sleep(2)
        if remote.ssh(f"sha256sum {remote.REMOTE_LIBRARY}").split()[0] != old_library_sha:
            raise RuntimeError("Failed to restore robot library")
        if remote.ssh(f"sha256sum {remote.REMOTE_YAML}").split()[0] != sha(old_yaml.encode()):
            raise RuntimeError("Failed to restore robot YAML")
        print("Previous robot program restored", flush=True)


if __name__ == "__main__":
    main()
