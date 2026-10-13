#!/usr/bin/env python3
"""Two-frequency C-car yaw identification from the current sweep-mode position.

Run in the RMCS development container with a fixed chassis, clear gimbal area,
and a remote operator. The active sweep-mode YAML and library are restored byte
for byte after the bounded, manually gated torque runs.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import time
from pathlib import Path

import run_chassis_stable_yaw_retest as base
import run_frequency_yaw_retest as frequency
import run_repeat_optimization as remote


ROOT = Path(__file__).resolve().parent
OUT = ROOT / "current_position_yaw_tuning_2026-10-13"
ACTIVE_YAML = "/rmcs_install/share/rmcs_bringup/config/sweep.yaml"
EXPECTED_ACTIVE_YAML_SHA = "3e02806428b1e95ac5c959aff3b8eef2017a53f93444b4fec457adc2339b9eb3"
EXPECTED_ACTIVE_LIBRARY_SHA = "97d034a6b7de07b59a3e2e292f5bd2ca52ea9fa26805f02cc95e64fbceb813c7"
PLAN = (("f3p125", 0.08, 10), ("f6p25", 0.04, 20))


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def without_chassis_control(config: bytes) -> bytes:
    """Leave bottom-board feedback available while removing chassis motor controllers.

    The same zero-output supercap stub is used by the robot's original sweep.yaml.
    All wheel and joint control topics remain without a producing component.
    """
    source = config.decode()
    lines = source.splitlines(keepends=True)
    disabled = 0
    for index, line in enumerate(lines):
        if line.startswith("      - rmcs_core::controller::chassis::"):
            lines[index] = line.replace("      -", "      # -", 1)
            disabled += 1
    if disabled < 3:
        raise RuntimeError("Expected active chassis components were not found")
    source = "".join(lines)
    component_marker = "      - rmcs_core::controller::gimbal::DeformableInfantryGimbalController -> gimbal_controller\n"
    if source.count(component_marker) != 1:
        raise RuntimeError("Unexpected gimbal controller component")
    source = source.replace(component_marker,
        "      - rmcs_core::controller::pid::PidController -> supercap_charge_limit_stub\n"
        + component_marker)
    section_marker = "\ngimbal_controller:\n"
    if source.count(section_marker) != 1:
        raise RuntimeError("Unexpected gimbal controller YAML section")
    source = source.replace(section_marker,
        "\nsupercap_charge_limit_stub:\n  ros__parameters:\n"
        "    measurement: 0.0\n    setpoint: 0.0\n"
        "    control: /chassis/supercap/charge_power_limit\n"
        "    kp: 0.0\n    ki: 0.0\n    kd: 0.0\n"
        + section_marker)
    if "      - rmcs_core::controller::chassis::" in source:
        raise RuntimeError("A chassis motor controller remains active")
    return source.encode()


def current_mode_is_sweep() -> bool:
    return any(
        line.startswith("/rmcs_install/lib/rmcs_executor/rmcs_executor ")
        and "config/sweep.yaml" in line
        for line in remote.ssh("ps -eo args").splitlines()
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--operator-ready", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=OUT)
    args = parser.parse_args()
    if not args.operator_ready:
        parser.error("The fixed chassis, clear area and remote operator must be confirmed")
    out = args.output_dir.resolve()
    if out.exists() and list(out.iterdir()):
        raise RuntimeError(f"Output directory is not empty: {out}")
    if remote.ssh("hostname") != "alliance-hero" or not current_mode_is_sweep():
        raise RuntimeError("The expected C-car sweep mode is not running")
    if base.remote_hash(ACTIVE_YAML) != EXPECTED_ACTIVE_YAML_SHA:
        raise RuntimeError("The running sweep YAML changed; leaving the robot untouched")
    if base.remote_hash(remote.REMOTE_LIBRARY) != EXPECTED_ACTIVE_LIBRARY_SHA:
        raise RuntimeError("The running library changed; leaving the robot untouched")
    if base.remote_hash(remote.REMOTE_YAML) != base.EXPECTED_YAML:
        raise RuntimeError("The archived normal C-car YAML does not match the robot")
    if sha(remote.LIBRARY.read_bytes()) != frequency.EXPECTED_TEST_LIBRARY:
        raise RuntimeError("The local test library differs from the reviewed build")

    before = base.BEFORE.read_bytes()
    if sha(before) != base.EXPECTED_YAML:
        raise RuntimeError("The archived C-car YAML changed")
    out.mkdir(parents=True)
    snapshot = out / "sweep_before.yaml"
    remote.call("scp", "-q", f"remote:{ACTIVE_YAML}", str(snapshot), timeout=30)
    if sha(snapshot.read_bytes()) != EXPECTED_ACTIVE_YAML_SHA:
        raise RuntimeError("The sweep YAML snapshot changed")
    stamp = str(int(time.time()))
    backup_yaml = f"/tmp/sweep_before_yaw_tuning_{stamp}.yaml"
    backup_library = f"/tmp/librmcs_core_before_yaw_tuning_{stamp}.so"
    remote.ssh(f"set -e; cp {ACTIVE_YAML} {backup_yaml}; "
               f"cp {remote.REMOTE_LIBRARY} {backup_library}")
    if (base.remote_hash(backup_yaml) != EXPECTED_ACTIVE_YAML_SHA
            or base.remote_hash(backup_library) != EXPECTED_ACTIVE_LIBRARY_SHA):
        raise RuntimeError("The remote backup failed verification")
    (out / "protocol.json").write_text(json.dumps({
        "purpose": "Refit J and B at the current yaw angle before controller tuning",
        "plan": [{"label": label, "phase_s": phase, "cycles": cycles,
                  "frequency_hz": 1 / (4 * phase), "torque_command_Nm": 0.7}
                 for label, phase, cycles in PLAN],
        "repeats_per_frequency": 2,
        "train": "run1 only", "holdout": "run2 only",
        "start_angle_tolerance_rad": 0.30,
        "note": "Shorter runs reduce post-run drift; the recorded start angles must still be checked before fitting",
        "chassis_components": "all chassis controllers disabled; feedback remains available",
        "baseline_sweep_yaml_sha256": EXPECTED_ACTIVE_YAML_SHA,
        "baseline_library_sha256": EXPECTED_ACTIVE_LIBRARY_SHA,
        "test_library_sha256": frequency.EXPECTED_TEST_LIBRARY,
        "remote_yaml_backup": backup_yaml,
        "remote_library_backup": backup_library,
    }, ensure_ascii=False, indent=2) + "\n")
    active = False
    manifest = []
    reference_angle = None
    try:
        if base.remote_hash("/tmp/librmcs_core_yaw_tuning.so") != frequency.EXPECTED_TEST_LIBRARY:
            remote.call("scp", "-q", str(remote.LIBRARY),
                        "remote:/tmp/librmcs_core_yaw_tuning.so", timeout=120)
        if base.remote_hash("/tmp/librmcs_core_yaw_tuning.so") != frequency.EXPECTED_TEST_LIBRARY:
            raise RuntimeError("Uploaded yaw test library has the wrong hash")
        for label, phase, cycles in PLAN:
            config = without_chassis_control(
                frequency.variant(before, phase, cycles, 0.7))
            config_file = out / f"{label}.yaml"
            config_file.write_bytes(config)
            old_logger = base.latest("yaw_[0-9]*.csv")
            remote.call("scp", "-q", str(config_file),
                        "remote:/tmp/yaw_tuning.yaml", timeout=30)
            remote.ssh("set -e; service rmcs stop; "
                       f"install -m 644 /tmp/librmcs_core_yaw_tuning.so {remote.REMOTE_LIBRARY}; "
                       f"install -m 644 /tmp/yaw_tuning.yaml {ACTIVE_YAML}; "
                       "service rmcs start", timeout=30)
            if (base.remote_hash(ACTIVE_YAML) != sha(config)
                    or base.remote_hash(remote.REMOTE_LIBRARY)
                    != frequency.EXPECTED_TEST_LIBRARY):
                raise RuntimeError("The test deployment failed hash verification")
            base.wait_for_service()
            for repeat in (1, 2):
                _, angle = base.wait_for_gate(reference_angle, 0.30,
                                              old_logger if repeat == 1 else "")
                if reference_angle is None:
                    reference_angle = angle
                previous = base.latest("yaw_plant_*.csv")
                active = True
                base.service("start")
                time.sleep(4 * phase * cycles + 0.65)
                robot_csv = base.latest("yaw_plant_*.csv")
                if not robot_csv or robot_csv == previous:
                    raise RuntimeError("No new yaw telemetry CSV")
                result = frequency.export_run(robot_csv, f"{label}_run{repeat}",
                                              phase, cycles, 0.7, out)
                active = False
                result.update(preflight_yaw_rad=angle, config_sha256=sha(config))
                manifest.append(result)
                (out / "manifest.json").write_text(json.dumps(
                    manifest, ensure_ascii=False, indent=2) + "\n")
                print(f"{label} run{repeat}: start={angle:.4f} rad, "
                      f"complete={result['completed']}, "
                      f"displacement={result['max_abs_displacement_rad']:.4f} rad, "
                      f"chassis_yaw={result['max_abs_chassis_yaw_delta_rad']:.4f} rad",
                      flush=True)
                if not result["completed"]:
                    raise RuntimeError("The bounded identification run ended early")
    except Exception:
        frequency.capture_log(out)
        raise
    finally:
        if active:
            try:
                base.service("stop")
            except Exception as exc:
                print(f"Could not call the yaw stop service: {exc}", flush=True)
        remote.ssh("set -e; service rmcs stop; "
                   f"install -m 644 {backup_library} {remote.REMOTE_LIBRARY}; "
                   f"install -m 644 {backup_yaml} {ACTIVE_YAML}; "
                   "service rmcs start", timeout=30)
        if (base.remote_hash(ACTIVE_YAML) != EXPECTED_ACTIVE_YAML_SHA
                or base.remote_hash(remote.REMOTE_LIBRARY) != EXPECTED_ACTIVE_LIBRARY_SHA
                or not current_mode_is_sweep()):
            raise RuntimeError("Original sweep mode was not restored exactly")
        print("Original sweep-mode YAML and library restored", flush=True)


if __name__ == "__main__":
    main()
