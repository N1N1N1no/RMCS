#!/usr/bin/env python3
"""Collect matched-amplitude yaw chirps, restoring the exact remote state.

Run from the RMCS development container after an operator confirms the physical
test area and explicitly allows the temporary C-car program switch.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import time
from pathlib import Path

import run_large_chirp_validation as chirp
import run_repeat_optimization as remote


ROOT = Path(__file__).resolve().parent
PRIMARY_PLAN = [
    ("01_low_004", 0.04, 0.3, 0.65),
    ("02_mid_004", 0.04, 1.0, 1.5),
    ("03_high_004", 0.04, 2.5, 3.0),
    ("04_low_004_repeat", 0.04, 0.3, 0.65),
    ("05_mid_004_repeat", 0.04, 1.0, 1.5),
    ("06_high_004_repeat", 0.04, 2.5, 3.0),
    ("07_low_008", 0.08, 0.3, 0.65),
]
FOLLOWUP_PLAN = [
    ("01_high_004_repeat", 0.04, 2.5, 3.0),
    ("02_low_008", 0.08, 0.3, 0.65),
    ("03_low_011", 0.11, 0.3, 0.65),
    ("04_mid_008", 0.08, 1.0, 1.5),
]


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def wait_for_gate(timeout_s: float = 15.0) -> None:
    """Wait for remote receiver startup without allowing any motion meanwhile."""
    deadline = time.monotonic() + timeout_s
    last_error = "no telemetry yet"
    while time.monotonic() < deadline:
        try:
            remote.check_gate(remote.latest_logger())
            return
        except RuntimeError as exc:
            last_error = str(exc)
            time.sleep(1)
    raise RuntimeError(f"Remote gate did not become ready in {timeout_s:g} s: {last_error}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--followup", action="store_true")
    args = parser.parse_args()
    global OUT
    OUT = ROOT / ("cross_frequency_followup_2026-10-06" if args.followup
                  else "cross_frequency_2026-10-06")
    plan = FOLLOWUP_PLAN if args.followup else PRIMARY_PLAN
    angle_tolerance = 0.12 if args.followup else 0.06
    if OUT.exists() and list(OUT.iterdir()):
        raise RuntimeError(f"Output directory must be empty: {OUT}")
    if remote.ssh("hostname") != "alliance-hero":
        raise RuntimeError("Unexpected SSH target")
    processes = remote.ssh("ps -eo args")
    if not any(line.startswith("/rmcs_install/lib/rmcs_executor/rmcs_executor ")
               and "deformable-infantry-omni-c.yaml" in line
               for line in processes.splitlines()):
        raise RuntimeError("C-car executor is not running")

    original_yaml = (remote.ssh(f"cat {remote.REMOTE_YAML}") + "\n").encode()
    original_lib_hash = remote.ssh(f"sha256sum {remote.REMOTE_LIBRARY}").split()[0]
    test_yaml = remote.SOURCE_YAML.read_text()
    test_library = remote.LIBRARY.read_bytes()
    test_library_hash = digest(test_library)
    if test_library_hash == original_lib_hash:
        raise RuntimeError("Expected a distinct compiled yaw-chirp library")
    if "yaw_chirp_amplitude_rad: 0.05" not in test_yaml:
        raise RuntimeError("Unexpected source yaw chirp configuration")

    OUT.mkdir(exist_ok=False)
    (OUT / "protocol.json").write_text(json.dumps({
        "plan": plan,
        "duration_s": 8.0,
        "torque_cap_Nm": 3.0,
        "displacement_stop_rad": 0.18,
        "speed_stop_rad_s": 1.2,
        "maximum_start_angle_drift_rad": angle_tolerance,
        "original_yaml_sha256": digest(original_yaml),
        "original_library_sha256": original_lib_hash,
        "test_yaml_sha256": digest(test_yaml.encode()),
        "test_library_sha256": test_library_hash,
        "source_cpp_sha256": digest((ROOT.parents[1] / chirp.CPP).read_bytes()),
    }, indent=2) + "\n")
    stamp = str(int(time.time()))
    backup_lib = f"/tmp/librmcs_core_before_cross_frequency_{stamp}.so"
    backup_yaml = f"/tmp/deformable_infantry_omni_c_before_cross_frequency_{stamp}.yaml"
    remote.ssh(f"set -e; cp {remote.REMOTE_LIBRARY} {backup_lib}; "
               f"cp {remote.REMOTE_YAML} {backup_yaml}")
    if remote.ssh(f"sha256sum {backup_lib}").split()[0] != original_lib_hash:
        raise RuntimeError("Remote library backup verification failed")
    if remote.ssh(f"sha256sum {backup_yaml}").split()[0] != digest(original_yaml):
        raise RuntimeError("Remote YAML backup verification failed")

    remote.call("scp", "-q", str(remote.LIBRARY),
                "remote:/tmp/librmcs_core_cross_frequency_test.so")
    manifest = []
    active = False
    chirp.RUN_DIR = OUT
    try:
        remote.ssh("set -e; service rmcs stop; "
                   f"install -m 644 /tmp/librmcs_core_cross_frequency_test.so {remote.REMOTE_LIBRARY}; "
                   "service rmcs start", timeout=30)
        time.sleep(2)
        if remote.ssh(f"sha256sum {remote.REMOTE_LIBRARY}").split()[0] != test_library_hash:
            raise RuntimeError("Test library deployment hash mismatch")
        for index, (label, amplitude, start_hz, end_hz) in enumerate(plan):
            config = chirp.variant(test_yaml, amplitude, start_hz, end_hz)
            remote.deploy_config(config, f"yaw-cross-frequency-{label}.yaml")
            wait_for_gate()
            previous = chirp.newest_chirp()
            print(f"Starting {label}: {amplitude:.2f} rad, {start_hz}-{end_hz} Hz", flush=True)
            active = True
            chirp.service("start")
            time.sleep(8.7)
            robot_path = chirp.newest_chirp()
            if not robot_path or robot_path == previous:
                raise RuntimeError(f"No fresh CSV after {label}")
            result = chirp.export_chirp(robot_path, label)
            active = False
            record = {"label": label, "amplitude_rad": amplitude,
                      "start_hz": start_hz, "end_hz": end_hz,
                      "config_sha256": digest(config.encode()), **result}
            manifest.append(record)
            (OUT / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
            print(f"Finished {label}: {result}", flush=True)
            if not result["completed"]:
                raise RuntimeError(f"Chirp ended early: {label}")
            if abs(result["start_angle_rad"] - manifest[0]["start_angle_rad"]) > angle_tolerance:
                raise RuntimeError("Yaw start angle drift exceeded matched-position limit")
            if index < len(plan) - 1 and (
                result["max_abs_displacement_rad"] > 0.16
                or result["max_abs_motor_speed_rad_s"] > 1.08
                or result["max_abs_command_torque_Nm"] > 2.80
            ):
                raise RuntimeError("Insufficient motion or torque headroom for next run")
    finally:
        if active:
            try:
                chirp.service("stop")
                time.sleep(1)
            except Exception as exc:
                print(f"Yaw chirp stop service failed: {exc}", flush=True)
        print("Restoring original robot program and YAML", flush=True)
        remote.ssh("set -e; service rmcs stop; "
                   f"install -m 644 {backup_lib} {remote.REMOTE_LIBRARY}; "
                   f"install -m 644 {backup_yaml} {remote.REMOTE_YAML}; "
                   "service rmcs start", timeout=30)
        time.sleep(2)
        if remote.ssh(f"sha256sum {remote.REMOTE_LIBRARY}").split()[0] != original_lib_hash:
            raise RuntimeError("Original robot library was not restored")
        if remote.ssh(f"sha256sum {remote.REMOTE_YAML}").split()[0] != digest(original_yaml):
            raise RuntimeError("Original robot YAML was not restored")
        print("Original robot program restored and verified", flush=True)


if __name__ == "__main__":
    main()
