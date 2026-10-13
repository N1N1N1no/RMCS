#!/usr/bin/env python3
"""Collect two matched C-car direct-torque runs, restoring the robot program.

Run inside the RMCS dev container with an operator holding the remote enable,
after the test area is confirmed clear. The temporary test library is restored.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import time
from pathlib import Path

import run_repeat_optimization as remote


ROOT = Path(__file__).resolve().parent
BEFORE = ROOT / "latency_2026-10-13/before.yaml"
TEST = ROOT / "latency_2026-10-13/test.yaml"
OUT = ROOT / "zero_delay_plant_retest_2026-10-13"
EXPECTED_LIBRARY = "8545d781b4ffeb4daf32f449cec15845c0bd09f775fbe8e362f24fd9721b80f4"
EXPECTED_YAML = "4f1974f583edb47c2e453461fabe1fb23fb87948605a445227c9cb946339c72a"
EXPECTED_TEST_YAML = "ed343ddd0801e03ba5ce8c2b1df33b5ef37fc829bc37d6d5cf2d6c993b9a7222"
EXPECTED_TEST_LIBRARY = "566dd082880bf26a6733115ec155fb34d83d156b1d4d3c18b882c322aedefcda"


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def remote_hash(path: str) -> str:
    return remote.ssh(f"sha256sum {path}").split()[0]


def latest(pattern: str) -> str:
    result = remote.ssh(
        f"find /tmp -maxdepth 1 -type f -name '{pattern}' "
        "-printf '%T@ %p\\n' | sort -nr | head -1"
    )
    if not result:
        return ""
    path = result.split(maxsplit=1)[1]
    if not path.startswith("/tmp/yaw_") or not path.endswith(".csv"):
        raise RuntimeError(f"Unexpected telemetry path: {path}")
    return path


def gate_row(path: str) -> dict[str, str]:
    lines = remote.ssh(f"head -1 {path}; tail -1 {path}").splitlines()
    if len(lines) != 2:
        raise RuntimeError("Yaw logger has no fresh sample")
    return next(csv.DictReader(lines))


def wait_for_gate(reference_angle: float | None, tolerance: float,
                  previous_logger: str = "") -> tuple[str, float]:
    deadline = time.monotonic() + 20
    last = "waiting for fresh telemetry"
    while time.monotonic() < deadline:
        try:
            logger = latest("yaw_[0-9]*.csv")
            if not logger or logger == previous_logger:
                raise RuntimeError(last)
            row = gate_row(logger)
            angle = float(row["/gimbal/yaw/angle"])
            velocity = float(row["/gimbal/yaw/velocity_imu"])
            if row["/gimbal/yaw_sweep/remote_ready"] != "1":
                raise RuntimeError("Remote enable is not held")
            if not math.isfinite(angle):
                raise RuntimeError("Nonfinite yaw angle")
            if reference_angle is not None and abs(angle - reference_angle) > tolerance:
                raise RuntimeError(f"Yaw {angle:.3f} rad is outside matched position")
            if not math.isfinite(velocity) or abs(velocity) > 0.10:
                raise RuntimeError(f"Yaw is moving at {velocity:.3f} rad/s")
            return logger, angle
        except (RuntimeError, ValueError, KeyError) as exc:
            last = str(exc)
            time.sleep(1)
    raise RuntimeError(f"Preflight gate did not become ready: {last}")


def service(action: str) -> str:
    response = remote.ssh(
        "bash -lc 'source /root/env_setup.bash && "
        f"ros2 service call /rmcs/service/yaw_plant/{action} "
        "std_srvs/srv/Trigger \"{}\"'",
        timeout=15,
    )
    if "success=True" not in response:
        raise RuntimeError(f"Yaw plant {action} refused: {response}")
    return response


def export_run(robot_path: str, label: str) -> dict:
    local = OUT / f"{label}.csv"
    remote.call("scp", "-q", f"remote:{robot_path}", str(local), timeout=30)
    with local.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError(f"Empty CSV: {robot_path}")
    t = [float(row["t_s"]) for row in rows]
    phase = [int(row["phase"]) for row in rows]
    angle = [float(row["angle_rad"]) for row in rows]
    velocity = [float(row["velocity_rad_s"]) for row in rows]
    if any(not math.isfinite(value) for value in t + angle + velocity):
        raise RuntimeError("Nonfinite yaw telemetry")
    if any(b <= a for a, b in zip(t, t[1:])):
        raise RuntimeError("Nonmonotonic yaw timestamps")
    result = {
        "label": label,
        "robot_csv": robot_path,
        "local_csv": str(local),
        "sha256": sha(local.read_bytes()),
        "samples": len(rows),
        "duration_s": t[-1],
        "last_phase": phase[-1],
        "start_angle_rad": angle[0],
        "max_abs_displacement_rad": max(abs(value - angle[0]) for value in angle),
        "max_abs_velocity_rad_s": max(abs(value) for value in velocity),
        "completed": len(rows) >= 3700 and t[-1] >= 3.80 and phase[-1] == 31,
    }
    return result


def main() -> None:
    global OUT
    parser = argparse.ArgumentParser()
    parser.add_argument("--operator-ready", action="store_true",
                        help="Area clear, chassis fixed, remote operator present")
    parser.add_argument("--reference-angle", type=float,
                        help="Optional reference yaw angle in rad for the first run")
    parser.add_argument("--angle-tolerance", type=float, default=0.18)
    parser.add_argument("--torque", type=float, default=0.9,
                        help="Direct yaw torque amplitude in N m, at most 0.9")
    parser.add_argument("--output-dir", type=Path, default=OUT)
    args = parser.parse_args()
    OUT = args.output_dir.resolve()
    if not args.operator_ready:
        parser.error("Physical test requires --operator-ready")
    if not 0 < args.angle_tolerance <= 0.30:
        parser.error("Angle tolerance must be in (0, 0.30] rad")
    if not 0.2 <= args.torque <= 0.9:
        parser.error("Torque must be in [0.2, 0.9] N m")
    if OUT.exists() and list(OUT.iterdir()):
        raise RuntimeError(f"Output directory must be empty: {OUT}")
    before, template = BEFORE.read_bytes(), TEST.read_bytes()
    if sha(before) != EXPECTED_YAML or sha(template) != EXPECTED_TEST_YAML:
        raise RuntimeError("Archived baseline or test YAML has changed")
    marker = "    # yaw_plant_torque_nm: 0.9"
    template_text = template.decode()
    if template_text.count(marker) != 1:
        raise RuntimeError("Expected yaw plant torque parameter is missing")
    test = template_text.replace(marker,
                                 f"    yaw_plant_torque_nm: {args.torque:g}").encode()
    test_yaml_hash = sha(test)
    if sha(remote.LIBRARY.read_bytes()) != EXPECTED_TEST_LIBRARY:
        raise RuntimeError("Compiled test library differs from reviewed build")
    if remote.ssh("hostname") != "alliance-hero":
        raise RuntimeError("Unexpected SSH target")
    if remote_hash(remote.REMOTE_LIBRARY) != EXPECTED_LIBRARY:
        raise RuntimeError("Robot library differs from verified baseline")
    if remote_hash(remote.REMOTE_YAML) != EXPECTED_YAML:
        raise RuntimeError("Robot YAML differs from verified baseline")
    processes = remote.ssh("ps -eo args")
    if not any(line.startswith("/rmcs_install/lib/rmcs_executor/rmcs_executor ")
               and "deformable-infantry-omni-c.yaml" in line
               for line in processes.splitlines()):
        raise RuntimeError("C-car executor is not running")

    OUT.mkdir()
    test_path = OUT / "test.yaml"
    test_path.write_bytes(test)
    stamp = str(int(time.time()))
    backup = f"/tmp/c_car_before_zero_delay_plant_{stamp}.yaml"
    backup_lib = f"/tmp/librmcs_core_before_zero_delay_plant_{stamp}.so"
    remote.ssh(f"set -e; cp {remote.REMOTE_YAML} {backup}; "
               f"cp {remote.REMOTE_LIBRARY} {backup_lib}")
    if remote_hash(backup) != EXPECTED_YAML or remote_hash(backup_lib) != EXPECTED_LIBRARY:
        raise RuntimeError("Robot backup verification failed")
    (OUT / "protocol.json").write_text(json.dumps({
        "reference_angle_rad": args.reference_angle,
        "angle_tolerance_rad": args.angle_tolerance,
        "torque_command_Nm": args.torque,
        "phase_s": 0.12,
        "cycles": 8,
        "baseline_library_sha256": EXPECTED_LIBRARY,
        "baseline_yaml_sha256": EXPECTED_YAML,
        "test_yaml_sha256": test_yaml_hash,
        "test_library_sha256": EXPECTED_TEST_LIBRARY,
        "remote_backup": backup,
        "remote_library_backup": backup_lib,
    }, indent=2) + "\n")
    manifest = []
    active = False
    try:
        previous_logger = latest("yaw_[0-9]*.csv")
        remote.call("scp", "-q", str(test_path),
                    "remote:/tmp/c_car_zero_delay_plant_test.yaml")
        remote.call("scp", "-q", str(remote.LIBRARY),
                    "remote:/tmp/librmcs_core_zero_delay_plant_test.so")
        remote.ssh("set -e; service rmcs stop; "
                   f"install -m 644 /tmp/librmcs_core_zero_delay_plant_test.so {remote.REMOTE_LIBRARY}; "
                   f"install -m 644 /tmp/c_car_zero_delay_plant_test.yaml {remote.REMOTE_YAML}; "
                   "service rmcs start", timeout=30)
        if (remote_hash(remote.REMOTE_YAML) != test_yaml_hash
                or remote_hash(remote.REMOTE_LIBRARY) != EXPECTED_TEST_LIBRARY):
            raise RuntimeError("Test program deployment hash mismatch")
        services = remote.ssh("bash -lc 'source /root/env_setup.bash && ros2 service list'",
                              timeout=15)
        if "/rmcs/service/yaw_plant/start" not in services.splitlines():
            raise RuntimeError("Yaw plant service missing from deployed test program")
        for index in (1, 2):
            reference = (args.reference_angle if index == 1
                         else manifest[0]["start_angle_rad"])
            tolerance = (args.angle_tolerance if index == 1
                         else min(args.angle_tolerance, 0.12))
            _, angle = wait_for_gate(reference, tolerance,
                                     previous_logger if index == 1 else "")
            previous = latest("yaw_plant_*.csv")
            active = True
            service("start")
            time.sleep(4.5)
            robot_path = latest("yaw_plant_*.csv")
            if not robot_path or robot_path == previous:
                raise RuntimeError("No new yaw plant CSV")
            result = export_run(robot_path, f"run{index}")
            active = False
            manifest.append(result)
            (OUT / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
            if not result["completed"]:
                raise RuntimeError(f"Yaw plant run {index} ended early: {result}")
            if abs(result["start_angle_rad"] - angle) > 0.08:
                raise RuntimeError("Start angle drifted after preflight")
    finally:
        if active:
            try:
                service("stop")
            except Exception as exc:
                print(f"Yaw plant stop failed: {exc}", flush=True)
        remote.ssh("set -e; service rmcs stop; "
                   f"install -m 644 {backup_lib} {remote.REMOTE_LIBRARY}; "
                   f"install -m 644 {backup} {remote.REMOTE_YAML}; "
                   "service rmcs start", timeout=30)
        if remote_hash(remote.REMOTE_YAML) != EXPECTED_YAML:
            raise RuntimeError("Original robot YAML was not restored")
        if remote_hash(remote.REMOTE_LIBRARY) != EXPECTED_LIBRARY:
            raise RuntimeError("Robot library changed during test")
        print("Original robot library and YAML verified", flush=True)
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
