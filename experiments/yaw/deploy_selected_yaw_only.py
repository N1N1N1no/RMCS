#!/usr/bin/env python3
"""Permanently enable the validated C-car yaw-only PID/FF configuration.

The previous on-robot library/YAML are backed up. On any failed post-check,
this script restores them before exiting with an error.
"""

from __future__ import annotations

import hashlib
import json
import time
from pathlib import Path

import run_repeat_optimization as remote


ROOT = Path(__file__).resolve().parent
RECORD_DIR = ROOT / "selected_yaw_only_deployment_2026-10-06"
EXPECTED_BEFORE_LIBRARY = "a9950e3b401b2240d1567f50dfa05a56cea006d3cf0411876ab1728d146155e6"
EXPECTED_BEFORE_YAML = "af5423346f8e7901e849c03acd4e6139c671ebd731df9b5d1390d4db95b2347c"
SELECTED = {
    "yaw_angle_kp": 12.0,
    "yaw_velocity_kp": 11.0,
    "yaw_velocity_ki": 0.05,
    "yaw_ref_velocity_gain": 1.0,
    "yaw_velocity_ff_gain": 1.378547846,
    "yaw_acceleration_ff_gain": 0.142611646,
    "yaw_sweep_feedforward_scale": 1.0,
    "yaw_sweep_torque_limit_nm": 4.0,
}


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def remote_hash(path: str) -> str:
    return remote.ssh(f"sha256sum {path}").split()[0]


def verify_source_yaml(config: str) -> None:
    for name, expected in SELECTED.items():
        line = f"    {name}: {expected}"
        if config.count(line) != 1:
            raise RuntimeError(f"Selected parameter missing or repeated: {line}")
    required = (
        "- rmcs_core::hardware::DeformableInfantryOmniC -> deformable_infantry",
        "- rmcs_core::controller::gimbal::DeformableInfantryGimbalController -> gimbal_controller",
        "# - rmcs_core::controller::chassis::DeformableOmniWheelController -> deformable_chassis_controller",
        "# - rmcs_core::controller::shooting::FrictionWheelController -> friction_wheel_controller",
    )
    for marker in required:
        if marker not in config:
            raise RuntimeError(f"Source YAML is not the expected yaw-only setup: {marker}")


def executor_running() -> bool:
    return any(line.startswith("/rmcs_install/lib/rmcs_executor/rmcs_executor ")
               and "deformable-infantry-omni-c.yaml" in line
               for line in remote.ssh("ps -eo args").splitlines())


def main() -> None:
    if RECORD_DIR.exists() and list(RECORD_DIR.iterdir()):
        raise RuntimeError(f"Deployment record already exists: {RECORD_DIR}")
    if remote.ssh("hostname") != "alliance-hero":
        raise RuntimeError("Unexpected robot host")
    if remote_hash(remote.REMOTE_LIBRARY) != EXPECTED_BEFORE_LIBRARY:
        raise RuntimeError("Robot library changed since preflight")
    if remote_hash(remote.REMOTE_YAML) != EXPECTED_BEFORE_YAML:
        raise RuntimeError("Robot YAML changed since preflight")
    if not executor_running():
        raise RuntimeError("Expected C-car executor is not running")

    source_yaml = remote.SOURCE_YAML.read_text()
    verify_source_yaml(source_yaml)
    target_library_hash = sha(remote.LIBRARY.read_bytes())
    target_yaml_hash = sha(source_yaml.encode())
    if target_library_hash != "e07d6255257ab870c9717f325b97cf13e68f6ecc9df5a9c75cc724486b1ad789":
        raise RuntimeError("Local compiled C-car library differs from tested artifact")

    RECORD_DIR.mkdir()
    original_yaml = RECORD_DIR / "before_deployment.yaml"
    remote.call("scp", "-q", f"remote:{remote.REMOTE_YAML}", str(original_yaml))
    if sha(original_yaml.read_bytes()) != EXPECTED_BEFORE_YAML:
        raise RuntimeError("Local backup YAML hash mismatch")
    stamp = str(int(time.time()))
    backup_lib = f"/var/tmp/librmcs_core_before_selected_yaw_{stamp}.so"
    backup_yaml = f"/var/tmp/deformable_infantry_omni_c_before_selected_yaw_{stamp}.yaml"
    remote.ssh(f"set -e; cp {remote.REMOTE_LIBRARY} {backup_lib}; "
               f"cp {remote.REMOTE_YAML} {backup_yaml}")
    if remote_hash(backup_lib) != EXPECTED_BEFORE_LIBRARY or remote_hash(backup_yaml) != EXPECTED_BEFORE_YAML:
        raise RuntimeError("Robot backup hash mismatch")

    remote.call("scp", "-q", str(remote.LIBRARY),
                "remote:/tmp/librmcs_core_selected_yaw.so")
    remote.call("scp", "-q", str(remote.SOURCE_YAML),
                "remote:/tmp/deformable_infantry_omni_c_selected_yaw.yaml")
    if remote_hash("/tmp/librmcs_core_selected_yaw.so") != target_library_hash:
        raise RuntimeError("Library upload hash mismatch")
    if remote_hash("/tmp/deformable_infantry_omni_c_selected_yaw.yaml") != target_yaml_hash:
        raise RuntimeError("YAML upload hash mismatch")

    try:
        remote.ssh("set -e; service rmcs stop; "
                   f"install -m 644 /tmp/librmcs_core_selected_yaw.so {remote.REMOTE_LIBRARY}; "
                   f"install -m 644 /tmp/deformable_infantry_omni_c_selected_yaw.yaml {remote.REMOTE_YAML}; "
                   "service rmcs start", timeout=30)
        time.sleep(3)
        if remote_hash(remote.REMOTE_LIBRARY) != target_library_hash:
            raise RuntimeError("Deployed library hash mismatch")
        if remote_hash(remote.REMOTE_YAML) != target_yaml_hash:
            raise RuntimeError("Deployed YAML hash mismatch")
        if not executor_running():
            raise RuntimeError("C-car yaw-only executor did not remain running")
        logger = remote.latest_logger()
        header = remote.ssh(f"head -1 {logger}")
        if "/gimbal/yaw_sweep/remote_ready" not in header:
            raise RuntimeError("Yaw-only logger did not start")
    except Exception:
        print("Deployment failed; restoring original library and YAML", flush=True)
        remote.ssh("set -e; service rmcs stop; "
                   f"install -m 644 {backup_lib} {remote.REMOTE_LIBRARY}; "
                   f"install -m 644 {backup_yaml} {remote.REMOTE_YAML}; "
                   "service rmcs start", timeout=30)
        time.sleep(2)
        if remote_hash(remote.REMOTE_LIBRARY) != EXPECTED_BEFORE_LIBRARY or remote_hash(remote.REMOTE_YAML) != EXPECTED_BEFORE_YAML:
            raise RuntimeError("Rollback did not restore original hashes")
        raise

    record = {
        "mode": "deformable-infantry-omni-c yaw-only",
        "robot_hostname": "alliance-hero",
        "selected_parameters": SELECTED,
        "before_library_sha256": EXPECTED_BEFORE_LIBRARY,
        "before_yaml_sha256": EXPECTED_BEFORE_YAML,
        "deployed_library_sha256": target_library_hash,
        "deployed_yaml_sha256": target_yaml_hash,
        "on_robot_library_backup": backup_lib,
        "on_robot_yaml_backup": backup_yaml,
        "yaw_logger": logger,
        "executor_running": True,
    }
    (RECORD_DIR / "deployment.json").write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps(record, indent=2), flush=True)


if __name__ == "__main__":
    main()
