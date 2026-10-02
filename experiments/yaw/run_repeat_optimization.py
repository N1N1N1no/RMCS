#!/usr/bin/env python3
"""Run interleaved, gated C-car yaw sweeps for feedforward tuning.

Execute inside the RMCS devcontainer after confirming the physical test area is clear.
The robot runs one 15 s sweep per arm; the original deployed C-car YAML is
restored at the end, including on a failed run when the remote is reachable.
"""

import argparse
import csv
import hashlib
import json
import math
import subprocess
import tempfile
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parent
SOURCE_YAML = ROOT.parents[1] / "rmcs_ws/src/rmcs_bringup/config/deformable-infantry-omni-c.yaml"
LIBRARY = ROOT.parents[1] / "rmcs_ws/install/lib/librmcs_core.so"
REMOTE_YAML = "/rmcs_install/share/rmcs_bringup/config/deformable-infantry-omni-c.yaml"
REMOTE_LIBRARY = "/rmcs_install/lib/librmcs_core.so"
RUN_DIR = ROOT / "repeat_2026-10-02"
ARMS = {
    "current": (1.378547846, 0.142611646),
    "candidate": (1.10, 0.171),
}
ORDER = ["current", "candidate"] * 3


def call(*args, timeout=30):
    result = subprocess.run(args, text=True, capture_output=True, timeout=timeout)
    if result.returncode:
        raise RuntimeError(f"{args!r} failed: {result.stderr.strip()} {result.stdout.strip()}")
    return result.stdout.strip()


def ssh(command, timeout=30):
    return call("ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", "remote", command,
                timeout=timeout)


def make_config(base, b_gain, j_gain):
    old_b = "yaw_velocity_ff_gain: 1.378547846"
    old_j = "yaw_acceleration_ff_gain: 0.142611646"
    if base.count(old_b) != 1 or base.count(old_j) != 1:
        raise ValueError("Source YAML gains are not the expected current values")
    return base.replace(old_b, f"yaw_velocity_ff_gain: {b_gain}").replace(
        old_j, f"yaw_acceleration_ff_gain: {j_gain}")


def deploy_config(text, remote_name):
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=True) as stream:
        stream.write(text)
        stream.flush()
        call("scp", "-q", stream.name, f"remote:/tmp/{remote_name}", timeout=30)
    expected = hashlib.sha256(text.encode()).hexdigest()
    ssh(f"set -e; service rmcs stop; install -m 644 /tmp/{remote_name} {REMOTE_YAML}; "
        "service rmcs start", timeout=30)
    time.sleep(2)
    remote_hash = ssh(f"sha256sum {REMOTE_YAML}").split()[0]
    if remote_hash != expected:
        raise RuntimeError("Deployed YAML hash mismatch")
    processes = ssh("ps -eo args")
    if not any(line.startswith("/rmcs_install/lib/rmcs_executor/rmcs_executor ")
               and "deformable-infantry-omni-c.yaml" in line for line in processes.splitlines()):
        raise RuntimeError("C-car executor is not running")


def latest_logger():
    path = ssh("ls -t /tmp/yaw_*.csv | head -1")
    if not path.startswith("/tmp/yaw_") or not path.endswith(".csv"):
        raise RuntimeError(f"Unexpected logger path: {path}")
    return path


def check_gate(path):
    tail = ssh(f"tail -1 {path}")
    with tempfile.TemporaryFile(mode="w+") as stream:
        stream.write(ssh(f"head -1 {path}") + "\n" + tail + "\n")
        stream.seek(0)
        row = next(csv.DictReader(stream))
    if row["/gimbal/yaw_sweep/remote_ready"] != "1":
        raise RuntimeError("Remote gate is not ready; no motion triggered")
    if row["/gimbal/yaw_sweep/active"] != "0":
        raise RuntimeError("Previous sweep still active")


def trigger():
    response = ssh("bash -lc 'source /root/env_setup.bash && "
                   "ros2 service call /rmcs/service/yaw_sweep/start "
                   "std_srvs/srv/Trigger \"{}\"'", timeout=15)
    if "success=True" not in response:
        raise RuntimeError(f"Sweep refused: {response}")


def export_completed(path, label):
    RUN_DIR.mkdir(exist_ok=True)
    raw_path = RUN_DIR / f"{label}.raw.csv"
    csv_path = RUN_DIR / f"{label}.csv"
    call("scp", "-q", f"remote:{path}", str(raw_path), timeout=30)
    latest = []
    current = []
    with raw_path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames
        for row in reader:
            if row["/gimbal/yaw_sweep/active"] == "1":
                current.append(row)
            elif current:
                latest, current = current, []
    raw_path.unlink()
    if current or not 2950 <= len(latest) <= 3050:
        raise RuntimeError(f"Incomplete sweep: {len(latest)} samples, still active={bool(current)}")
    indices = [int(row["index"]) for row in latest]
    if any(b - a != 1 for a, b in zip(indices, indices[1:])):
        raise RuntimeError("Noncontinuous logger indices")
    angles = [float(row["/gimbal/yaw/angle"]) for row in latest]
    speeds = [abs(float(row["/gimbal/yaw/velocity_imu"])) for row in latest]
    torques = [abs(float(row["/gimbal/yaw/control_torque"])) for row in latest]
    errors = [abs(float(row["/gimbal/yaw/control_angle_error"])) for row in latest]
    if max(angles) - min(angles) > 0.30 or max(speeds) > 1.2 or max(torques) > 4.5:
        raise RuntimeError("Motion or torque exceeded experiment bounds")
    if not all(math.isfinite(v) for v in angles + speeds + torques + errors):
        raise RuntimeError("Nonfinite telemetry")
    with csv_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(latest)
    return {"samples": len(latest), "max_speed_rad_s": max(speeds),
            "max_command_torque_Nm": max(torques), "max_error_deg": math.degrees(max(errors)),
            "csv": str(csv_path)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidate-b", type=float, default=1.10)
    parser.add_argument("--candidate-j", type=float, default=0.171)
    parser.add_argument("--run-dir", default="repeat_2026-10-02")
    args = parser.parse_args()
    if not 0.0 <= args.candidate_b <= 2.0 or not 0.0 <= args.candidate_j <= 0.3:
        raise ValueError("Candidate gains outside the experiment bounds")
    global RUN_DIR
    RUN_DIR = ROOT / args.run_dir
    ARMS["candidate"] = (args.candidate_b, args.candidate_j)
    if ssh("hostname") != "alliance-hero":
        raise RuntimeError("Unexpected SSH target")
    local_hash = hashlib.sha256(LIBRARY.read_bytes()).hexdigest()
    if ssh(f"sha256sum {REMOTE_LIBRARY}").split()[0] != local_hash:
        raise RuntimeError("Robot library differs from compiled local library")
    deployed = ssh(f"cat {REMOTE_YAML}") + "\n"
    base = SOURCE_YAML.read_text()
    if deployed != base:
        raise RuntimeError("Robot C-car YAML differs from local selected configuration")
    RUN_DIR.mkdir(exist_ok=True)
    manifest = []
    in_motion = False
    try:
        for number, arm in enumerate(ORDER, start=1):
            label = f"{number:02d}_{arm}"
            b_gain, j_gain = ARMS[arm]
            print(f"Starting {label}: B={b_gain}, J={j_gain}", flush=True)
            deploy_config(make_config(base, b_gain, j_gain), f"yaw-repeat-{label}.yaml")
            logger = latest_logger()
            check_gate(logger)
            trigger()
            in_motion = True
            time.sleep(16.5)
            result = export_completed(logger, label)
            in_motion = False
            manifest.append({"label": label, "arm": arm, "B": b_gain, "J": j_gain,
                             "robot_logger": logger, **result})
            (RUN_DIR / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
            print(f"Finished {label}: {result}", flush=True)
    finally:
        if in_motion:
            try:
                ssh("bash -lc 'source /root/env_setup.bash && "
                    "ros2 service call /rmcs/service/yaw_sweep/stop "
                    "std_srvs/srv/Trigger \"{}\"'", timeout=15)
                time.sleep(1)
            except Exception as exc:
                print(f"Could not stop active sweep: {exc}", flush=True)
        print("Restoring original C-car YAML", flush=True)
        deploy_config(base, "yaw-repeat-original.yaml")


if __name__ == "__main__":
    main()
