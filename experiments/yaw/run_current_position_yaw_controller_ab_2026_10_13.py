#!/usr/bin/env python3
"""Bounded, paired yaw PID/feedforward A/B at the newly identified position.

The current sweep-mode YAML and library are backed up and restored exactly.
Run only with the fixed chassis, clear gimbal area and remote operator.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import time
from pathlib import Path

import run_chassis_stable_yaw_retest as base
import run_current_position_yaw_tuning_2026_10_13 as identify
import run_frequency_yaw_retest as frequency
import run_repeat_optimization as remote


ROOT = Path(__file__).resolve().parent
MODEL_J = 0.11221803719098242
MODEL_B = 1.781243415052695
ARMS = {
    "old_robot": (10.0, 13.0, 0.02, 0.13, 0.18),
    "baseline": (12.0, 11.0, 0.05, 1.378547846, 0.142611646),
    "newff": (12.0, 11.0, 0.05, MODEL_B, MODEL_J),
    "angle14": (14.0, 11.0, 0.05, MODEL_B, MODEL_J),
    "speed13": (12.0, 13.0, 0.05, MODEL_B, MODEL_J),
    "angle14_speed13": (14.0, 13.0, 0.05, MODEL_B, MODEL_J),
    "ki08": (12.0, 11.0, 0.08, MODEL_B, MODEL_J),
}
GAINS = (
    ("yaw_angle_kp", "10.0"),
    ("yaw_velocity_kp", "13.0"),
    ("yaw_velocity_ki", "0.02"),
    ("yaw_velocity_ff_gain", "0.13"),
    ("yaw_acceleration_ff_gain", "0.18"),
)
SIGNALS = (
    "/gimbal/yaw_sweep/active", "/gimbal/yaw_sweep/target_offset",
    "/gimbal/yaw_sweep/reference_velocity", "/gimbal/yaw_sweep/reference_acceleration",
    "/gimbal/yaw_sweep/feedforward_scale", "/gimbal/yaw_sweep/remote_ready",
    "/gimbal/yaw_sweep/right_switch", "/gimbal/yaw/angle", "/gimbal/yaw/velocity_imu",
    "/gimbal/yaw/control_angle_error", "/gimbal/yaw/control_torque", "/gimbal/yaw/torque",
    "/chassis/yaw/velocity_imu", *base.CHASSIS_SIGNALS,
)


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def make_config(before: bytes, arm: str) -> bytes:
    source = identify.without_chassis_control(
        frequency.variant(before, 0.08, 10, 0.7)).decode()
    marker = "    yaw_plant_cycles: 10\n"
    if source.count(marker) != 1:
        raise RuntimeError("Unexpected yaw test YAML")
    source = source.replace(marker, marker
        + "    yaw_sweep_amplitude_rad: 0.06\n"
        + "    yaw_sweep_hold_seconds: 1.5\n"
        + "    yaw_sweep_cycles: 4\n"
        + "    yaw_sweep_transition_seconds: 0.5\n"
        + "    yaw_sweep_feedforward_scale: 1.0\n"
        + "    yaw_sweep_torque_limit_nm: 2.5\n")
    start = source.index("\nvalue_collector:\n") + 1
    end = source.index("value_broadcaster:\n", start)
    collector = ("value_collector:\n  ros__parameters:\n"
                 "    csv_path: \"/tmp/yaw_<timestamp>.csv\"\n    signals:\n")
    collector += "".join(f"      - {signal}\n" for signal in SIGNALS)
    collector += "    write_interval: 5\n    flush_interval: 100\n\n"
    source = source[:start] + collector + source[end:]
    for (key, old), value in zip(GAINS, ARMS[arm]):
        original = f"    {key}: {old}\n"
        if source.count(original) != 1:
            raise RuntimeError(f"Unexpected C-car control gain {key}")
        source = source.replace(original, f"    {key}: {value}\n")
    return source.encode()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--operator-ready", action="store_true")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--order", required=True,
                        help="Comma-separated arm names, for example baseline,newff,newff,baseline")
    parser.add_argument("--expected-start", type=float, default=5.67,
                        help="Preflight yaw center in rad; the gate allows only ±0.15 rad")
    args = parser.parse_args()
    if not args.operator_ready:
        parser.error("Physical fixture, clear area and remote operator required")
    order = args.order.split(",")
    if not order or any(arm not in ARMS for arm in order):
        parser.error(f"Arms must be selected from {sorted(ARMS)}")
    out = args.output_dir.resolve()
    if out.exists() and list(out.iterdir()):
        raise RuntimeError(f"Output directory is not empty: {out}")
    if remote.ssh("hostname") != "alliance-hero" or not identify.current_mode_is_sweep():
        raise RuntimeError("Expected C-car sweep mode is not running")
    if (base.remote_hash(identify.ACTIVE_YAML) != identify.EXPECTED_ACTIVE_YAML_SHA
            or base.remote_hash(remote.REMOTE_LIBRARY)
            != identify.EXPECTED_ACTIVE_LIBRARY_SHA
            or base.remote_hash(remote.REMOTE_YAML) != base.EXPECTED_YAML):
        raise RuntimeError("Robot software differs from the checked baseline")
    if sha(remote.LIBRARY.read_bytes()) != frequency.EXPECTED_TEST_LIBRARY:
        raise RuntimeError("Local yaw test library is not the reviewed build")
    before = base.BEFORE.read_bytes()
    if sha(before) != base.EXPECTED_YAML:
        raise RuntimeError("Archived C-car YAML changed")
    out.mkdir(parents=True)
    backup_yaml = f"/tmp/sweep_before_yaw_ab_{int(time.time())}.yaml"
    backup_library = f"/tmp/librmcs_core_before_yaw_ab_{int(time.time())}.so"
    remote.ssh(f"set -e; cp {identify.ACTIVE_YAML} {backup_yaml}; "
               f"cp {remote.REMOTE_LIBRARY} {backup_library}")
    if (base.remote_hash(backup_yaml) != identify.EXPECTED_ACTIVE_YAML_SHA
            or base.remote_hash(backup_library) != identify.EXPECTED_ACTIVE_LIBRARY_SHA):
        raise RuntimeError("Robot backup verification failed")
    (out / "protocol.json").write_text(json.dumps({
        "order": order,
        "arms": {arm: dict(zip(("angle_kp", "velocity_kp", "velocity_ki",
                                "velocity_ff", "acceleration_ff"), ARMS[arm]))
                 for arm in set(order)},
        "trajectory": {"amplitude_rad": 0.06, "transition_s": 0.5,
                       "hold_s": 1.5, "cycles": 4},
        "torque_command_limit_Nm": 2.5,
        "chassis_components": "all chassis controllers disabled; feedback remains available",
        "reference_start_yaw_rad": [5.319, 5.393],
        "preflight_yaw_center_rad": args.expected_start,
        "accepted_start_tolerance_rad": 0.15,
        "source_model": str(identify.OUT.parent / "current_position_yaw_tuning_chassis_disabled_retry_2026-10-13/analysis.json"),
        "robot_yaml_backup": backup_yaml,
        "robot_library_backup": backup_library,
    }, ensure_ascii=False, indent=2) + "\n")
    manifest = []
    active = False
    remote.RUN_DIR = out
    try:
        if base.remote_hash("/tmp/librmcs_core_yaw_ab.so") != frequency.EXPECTED_TEST_LIBRARY:
            remote.call("scp", "-q", str(remote.LIBRARY),
                        "remote:/tmp/librmcs_core_yaw_ab.so", timeout=120)
        if base.remote_hash("/tmp/librmcs_core_yaw_ab.so") != frequency.EXPECTED_TEST_LIBRARY:
            raise RuntimeError("Uploaded yaw test library has the wrong hash")
        for index, arm in enumerate(order, 1):
            config = make_config(before, arm)
            config_file = out / f"{index:02d}_{arm}.yaml"
            config_file.write_bytes(config)
            previous_logger = base.latest("yaw_[0-9]*.csv")
            remote.call("scp", "-q", str(config_file), "remote:/tmp/yaw_ab.yaml", timeout=30)
            remote.ssh("set -e; service rmcs stop; "
                       f"install -m 644 /tmp/librmcs_core_yaw_ab.so {remote.REMOTE_LIBRARY}; "
                       f"install -m 644 /tmp/yaw_ab.yaml {identify.ACTIVE_YAML}; "
                       "service rmcs start", timeout=30)
            if (base.remote_hash(identify.ACTIVE_YAML) != sha(config)
                    or base.remote_hash(remote.REMOTE_LIBRARY)
                    != frequency.EXPECTED_TEST_LIBRARY):
                raise RuntimeError("A/B deployment failed hash verification")
            base.wait_for_service()
            logger, angle = base.wait_for_gate(args.expected_start, 0.15, previous_logger)
            remote.check_gate(logger)
            label = f"{index:02d}_{arm}"
            print(f"Starting {label} at {angle:.4f} rad", flush=True)
            remote.trigger()
            active = True
            time.sleep(16.5)
            result = remote.export_completed(logger, label, transfer_timeout=120)
            active = False
            if result["max_speed_rad_s"] > 1.0 or result["max_command_torque_Nm"] > 2.5:
                raise RuntimeError(f"A/B run has little motion or torque headroom: {label}")
            manifest.append({"label": label, "arm": arm, "start_angle_rad": angle,
                             "config_sha256": sha(config), **result})
            (out / "manifest.json").write_text(json.dumps(
                manifest, ensure_ascii=False, indent=2) + "\n")
            print(f"Finished {label}: {result}", flush=True)
    except Exception:
        frequency.capture_log(out)
        raise
    finally:
        if active:
            try:
                remote.ssh("bash -lc 'source /root/env_setup.bash && "
                           "ros2 service call /rmcs/service/yaw_sweep/stop "
                           "std_srvs/srv/Trigger \"{}\"'", timeout=15)
            except Exception as exc:
                print(f"Could not call sweep stop service: {exc}", flush=True)
        remote.ssh("set -e; service rmcs stop; "
                   f"install -m 644 {backup_library} {remote.REMOTE_LIBRARY}; "
                   f"install -m 644 {backup_yaml} {identify.ACTIVE_YAML}; "
                   "service rmcs start", timeout=30)
        if (base.remote_hash(identify.ACTIVE_YAML) != identify.EXPECTED_ACTIVE_YAML_SHA
                or base.remote_hash(remote.REMOTE_LIBRARY)
                != identify.EXPECTED_ACTIVE_LIBRARY_SHA
                or not identify.current_mode_is_sweep()):
            raise RuntimeError("Original sweep mode was not restored exactly")
        print("Original sweep-mode YAML and library restored", flush=True)


if __name__ == "__main__":
    main()
