#!/usr/bin/env python3
"""C-car bounded yaw torque frequency test, preserving original chassis enable.

Run inside the RMCS development container with the confirmed physical fixture
and remote operator. Each configuration is a four-phase zero-mean waveform;
its fundamental frequency is 1/(4*phase_seconds). Original files are restored.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import time
from pathlib import Path

import run_chassis_stable_yaw_retest as base
import run_repeat_optimization as remote


ROOT = Path(__file__).resolve().parent
OUT = ROOT / "frequency_yaw_retest_2026-10-13"
EXPECTED_TEST_LIBRARY = "3da87769467705dd8cf6b88a293ab705463856935136991a752d44b5732790aa"
PLAN = (
    # label, phase seconds, four-phase cycles, command Nm
    ("f2p083", 0.12, 12, 0.5),
    ("f3p125", 0.08, 20, 0.5),
    ("f6p25", 0.04, 40, 0.5),
    ("f7p5", 1.0 / 30.0, 48, 0.5),
    ("f8p333", 0.03, 53, 0.5),
    ("f10", 0.025, 64, 0.5),
    ("f20", 0.0125, 128, 0.5),
    ("f1p25", 0.20, 8, 0.5),
    ("f0p625", 0.40, 4, 0.5),
)


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def variant(before: bytes, phase_s: float, cycles: int, torque: float) -> bytes:
    source = before.decode()
    # Direct-torque identification must not simultaneously run the aim pipeline.
    for component in ("AutoAimRecorderComponent -> auto_aim_recorder",
                      "AutoAimComponent -> auto_aim_component",
                      "AutoAimCapturerComponent -> auto_aim_capturer"):
        enabled = "      - rmcs::" + component
        source = source.replace(enabled, enabled.replace("      -", "      # -"))
    enabled_ui = "      - rmcs_core::referee::app::ui::AutoAimUi -> auto_aim_ui"
    source = source.replace(enabled_ui, enabled_ui.replace("      -", "      # -"))
    config = base.make_test_yaml(source.encode(), torque).decode()
    marker = f"    yaw_plant_torque_nm: {torque:g}\n"
    if config.count(marker) != 1:
        raise RuntimeError("Test torque parameter missing")
    config = config.replace(marker, marker
                            + f"    yaw_plant_phase_seconds: {phase_s:g}\n"
                            + f"    yaw_plant_cycles: {cycles}\n")
    return config.encode()


def export_run(path: str, label: str, phase_s: float, cycles: int,
               torque: float, out: Path) -> dict:
    local = out / f"{label}.csv"
    remote.call("scp", "-q", f"remote:{path}", str(local), timeout=30)
    with local.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError(f"Empty yaw CSV: {path}")
    def values(name: str) -> list[float]:
        result = [float(row[name]) for row in rows]
        if any(not math.isfinite(x) for x in result):
            raise RuntimeError(f"Nonfinite {name} in {label}")
        return result
    t = values("t_s")
    if any(b <= a for a, b in zip(t, t[1:])):
        raise RuntimeError(f"Nonmonotonic time in {label}")
    angle = values("angle_rad")
    velocity = values("velocity_rad_s")
    chassis_yaw = values("chassis_yaw_delta_rad")
    state = {
        name: max(abs(x - values(name)[0]) for x in values(name))
        for name in ("chassis_pitch_rad", "chassis_roll_rad", "lf_joint_rad",
                     "lb_joint_rad", "rb_joint_rad", "rf_joint_rad")
    }
    duration = 4 * phase_s * cycles
    result = {
        "label": label,
        "robot_csv": path,
        "local_csv": str(local),
        "sha256": sha(local.read_bytes()),
        "frequency_hz": 1.0 / (4 * phase_s),
        "phase_s": phase_s,
        "cycles": cycles,
        "torque_command_Nm": torque,
        "samples": len(rows),
        "duration_s": t[-1],
        "last_phase": int(rows[-1]["phase"]),
        "start_angle_rad": angle[0],
        "max_abs_displacement_rad": max(abs(x - angle[0]) for x in angle),
        "max_abs_encoder_velocity_rad_s": max(map(abs, velocity)),
        "max_abs_chassis_yaw_delta_rad": max(map(abs, chassis_yaw)),
        "max_abs_chassis_yaw_rate_rad_s": max(map(abs, values("chassis_imu_rad_s"))),
        "max_abs_chassis_state_change_rad": state,
        "completed": (len(rows) >= int(0.9 * 1000 * duration)
                      and t[-1] >= duration - 0.005
                      and int(rows[-1]["phase"]) == 4 * cycles - 1),
    }
    return result


def capture_log(out: Path) -> None:
    try:
        recent = remote.ssh(
            "find /root/.ros/log -mindepth 2 -maxdepth 2 -type f "
            "-name launch.log -printf '%T@ %p\\n' | sort -nr | head -1"
        )
        path = recent.split(maxsplit=1)[1]
        if not path.startswith("/root/.ros/log/") or not path.endswith("/launch.log"):
            raise RuntimeError("Unexpected ROS log path")
        remote.call("scp", "-q", f"remote:{path}", str(out / "runtime.log"), timeout=20)
    except Exception as exc:
        print(f"Could not collect ROS log: {exc}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--operator-ready", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=OUT)
    parser.add_argument("--labels", help="Comma-separated subset of planned frequency labels")
    parser.add_argument("--repeats", type=int, choices=(1, 2), default=2)
    parser.add_argument("--torque-override", type=float,
                        help="Override planned torque, in [0.2, 0.7] N m")
    parser.add_argument("--match-start-tolerance", type=float,
                        help="Keep every run within this many radians of the first start angle")
    parser.add_argument("--use-robot-config", action="store_true",
                        help="Snapshot the current robot YAML and restore it after testing")
    parser.add_argument("--expected-robot-yaml-sha",
                        help="Require this current robot YAML SHA-256 before testing")
    args = parser.parse_args()
    if not args.operator_ready:
        parser.error("Physical fixture, clear area and remote operator must be confirmed")
    labels = set(args.labels.split(",")) if args.labels else {entry[0] for entry in PLAN}
    if labels - {entry[0] for entry in PLAN}:
        parser.error(f"Unknown frequency labels: {sorted(labels - {entry[0] for entry in PLAN})}")
    if args.torque_override is not None and not 0.2 <= args.torque_override <= 0.7:
        parser.error("Torque override must be in [0.2, 0.7] N m")
    if args.match_start_tolerance is not None and not 0 < args.match_start_tolerance <= 0.18:
        parser.error("Start angle tolerance must be in (0, 0.18] rad")
    plan = tuple((label, phase, cycles,
                  args.torque_override if args.torque_override is not None else torque)
                 for label, phase, cycles, torque in PLAN if label in labels)
    out = args.output_dir.resolve()
    if out.exists() and list(out.iterdir()):
        raise RuntimeError(f"Output directory is not empty: {out}")
    if args.use_robot_config != bool(args.expected_robot_yaml_sha):
        parser.error("--use-robot-config requires --expected-robot-yaml-sha and vice versa")
    if sha(remote.LIBRARY.read_bytes()) != EXPECTED_TEST_LIBRARY:
        raise RuntimeError("Compiled frequency test library differs from reviewed build")
    if remote.ssh("hostname") != "alliance-hero":
        raise RuntimeError("Unexpected robot")
    expected_yaml = args.expected_robot_yaml_sha or base.EXPECTED_YAML
    if base.remote_hash(remote.REMOTE_YAML) != expected_yaml:
        raise RuntimeError("Robot YAML differs from baseline")
    if base.remote_hash(remote.REMOTE_LIBRARY) != base.EXPECTED_LIBRARY:
        raise RuntimeError("Robot library differs from baseline")
    processes = remote.ssh("ps -eo args")
    if not any(line.startswith("/rmcs_install/lib/rmcs_executor/rmcs_executor ")
               and "deformable-infantry-omni-c.yaml" in line
               for line in processes.splitlines()):
        raise RuntimeError("C-car executor not running")

    out.mkdir()
    if args.use_robot_config:
        snapshot = out / "robot_config_before.yaml"
        remote.call("scp", "-q", f"remote:{remote.REMOTE_YAML}", str(snapshot), timeout=30)
        before = snapshot.read_bytes()
    else:
        before = base.BEFORE.read_bytes()
    if sha(before) != expected_yaml:
        raise RuntimeError("Baseline YAML changed during preflight")
    stamp = str(int(time.time()))
    backup_yaml = f"/tmp/c_car_before_frequency_yaw_{stamp}.yaml"
    backup_lib = f"/tmp/librmcs_core_before_frequency_yaw_{stamp}.so"
    remote.ssh(f"set -e; cp {remote.REMOTE_YAML} {backup_yaml}; "
               f"cp {remote.REMOTE_LIBRARY} {backup_lib}")
    if base.remote_hash(backup_yaml) != expected_yaml or base.remote_hash(backup_lib) != base.EXPECTED_LIBRARY:
        raise RuntimeError("Robot backup hash mismatch")
    (out / "protocol.json").write_text(json.dumps({
        "plan": [
            {"label": label, "phase_s": phase, "cycles": cycles,
             "frequency_hz": 1 / (4 * phase), "torque_Nm": torque}
            for label, phase, cycles, torque in plan
        ],
        "repeats": args.repeats,
        "match_start_tolerance_rad": args.match_start_tolerance,
        "sampling_rate_hz": 1000,
        "original_chassis_components_enabled": True,
        "chassis_guard_rad": 0.05,
        "chassis_yaw_delta_guard_rad": 0.03,
        "baseline_yaml_sha256": expected_yaml,
        "baseline_library_sha256": base.EXPECTED_LIBRARY,
        "test_library_sha256": EXPECTED_TEST_LIBRARY,
        "robot_backup_yaml": backup_yaml,
        "robot_backup_library": backup_lib,
    }, indent=2) + "\n")
    active = False
    manifest = []
    reference_angle = None
    try:
        remote.call("scp", "-q", str(remote.LIBRARY),
                    "remote:/tmp/librmcs_core_frequency_test.so", timeout=30)
        for label, phase, cycles, torque in plan:
            config = variant(before, phase, cycles, torque)
            config_path = out / f"{label}.yaml"
            config_path.write_bytes(config)
            previous_logger = base.latest("yaw_[0-9]*.csv")
            remote.call("scp", "-q", str(config_path),
                        "remote:/tmp/c_car_frequency_test.yaml", timeout=30)
            remote.ssh("set -e; service rmcs stop; "
                       "install -m 644 /tmp/librmcs_core_frequency_test.so "
                       f"{remote.REMOTE_LIBRARY}; "
                       "install -m 644 /tmp/c_car_frequency_test.yaml "
                       f"{remote.REMOTE_YAML}; service rmcs start", timeout=30)
            if base.remote_hash(remote.REMOTE_LIBRARY) != EXPECTED_TEST_LIBRARY:
                raise RuntimeError("Test library deployment mismatch")
            if base.remote_hash(remote.REMOTE_YAML) != sha(config):
                raise RuntimeError("Test config deployment mismatch")
            base.wait_for_service()
            duration = 4 * phase * cycles
            for repeat in range(1, args.repeats + 1):
                _, angle = base.wait_for_gate(reference_angle, args.match_start_tolerance or 0.18,
                                              previous_logger if repeat == 1 else "")
                if reference_angle is None and args.match_start_tolerance is not None:
                    reference_angle = angle
                previous = base.latest("yaw_plant_*.csv")
                active = True
                base.service("start")
                time.sleep(duration + 0.65)
                robot_path = base.latest("yaw_plant_*.csv")
                if not robot_path or robot_path == previous:
                    raise RuntimeError(f"No new yaw CSV for {label} run {repeat}")
                result = export_run(robot_path, f"{label}_run{repeat}", phase,
                                    cycles, torque, out)
                active = False
                result["preflight_yaw_rad"] = angle
                result["config_sha256"] = sha(config)
                manifest.append(result)
                (out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
                print(f"{label} run {repeat}: {result['samples']} samples, "
                      f"completed={result['completed']}, "
                      f"displacement={result['max_abs_displacement_rad']:.4f} rad, "
                      f"chassis yaw={result['max_abs_chassis_yaw_delta_rad']:.4f} rad",
                      flush=True)
                if not result["completed"]:
                    raise RuntimeError(f"{label} run {repeat} stopped before completion")
    except Exception:
        capture_log(out)
        raise
    finally:
        if active:
            try:
                base.service("stop")
            except Exception as exc:
                print(f"Yaw stop service failed: {exc}", flush=True)
        remote.ssh("set -e; service rmcs stop; "
                   f"install -m 644 {backup_lib} {remote.REMOTE_LIBRARY}; "
                   f"install -m 644 {backup_yaml} {remote.REMOTE_YAML}; "
                   "service rmcs start", timeout=30)
        if base.remote_hash(remote.REMOTE_LIBRARY) != base.EXPECTED_LIBRARY:
            raise RuntimeError("Original library not restored")
        if base.remote_hash(remote.REMOTE_YAML) != expected_yaml:
            raise RuntimeError("Original YAML not restored")
        print("Original C-car library and YAML restored and verified", flush=True)


if __name__ == "__main__":
    main()
