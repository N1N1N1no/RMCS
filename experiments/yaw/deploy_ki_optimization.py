#!/usr/bin/env python3
"""Deploy only the validated C-car yaw velocity Ki change; roll back on failure."""

import hashlib
import json
import subprocess
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path

import run_repeat_optimization as remote


ROOT = Path(__file__).resolve().parent
OUTPUT = ROOT / "ki_optimization_2026-10-02" / "deployment.json"
YAML = "rmcs_ws/src/rmcs_bringup/config/deformable-infantry-omni-c.yaml"
EXPECTED_LIBRARY = "2a046d42b9b5b389691688ffe89263a221e82579e9ac425ccf015f51d7431b3c"


def sha(data):
    return hashlib.sha256(data.encode()).hexdigest()


def deploy(text, label):
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=True) as stream:
        stream.write(text)
        stream.flush()
        remote.call("scp", "-q", stream.name, f"remote:/tmp/{label}.yaml", timeout=30)
    remote.ssh("set -e; service rmcs stop; "
               f"install -m 644 /tmp/{label}.yaml {remote.REMOTE_YAML}; "
               "service rmcs start", timeout=30)
    for _ in range(20):
        actual = remote.ssh(f"sha256sum {remote.REMOTE_YAML}").split()[0]
        processes = remote.ssh("ps -eo args")
        running = any(line.startswith("/rmcs_install/lib/rmcs_executor/rmcs_executor ")
                      and "deformable-infantry-omni-c.yaml" in line
                      for line in processes.splitlines())
        if actual == sha(text) and running:
            return
        time.sleep(.5)
    raise RuntimeError(f"C-car deployment verification failed: hash {actual}, running {running}")


def main():
    if remote.ssh("hostname") != "alliance-hero":
        raise RuntimeError("Unexpected SSH target")
    old = subprocess.check_output(["git", "show", f"HEAD:{YAML}"]).decode()
    if remote.ssh(f"cat {remote.REMOTE_YAML}") + "\n" != old:
        raise RuntimeError("Robot YAML is not the pre-test C-car configuration")
    if remote.ssh(f"sha256sum {remote.REMOTE_LIBRARY}").split()[0] != EXPECTED_LIBRARY:
        raise RuntimeError("Robot library is not the expected C-car build")
    marker = "yaw_velocity_ki: 0.02"
    if old.count(marker) != 1:
        raise RuntimeError("Unexpected original yaw velocity Ki")
    candidate = old.replace(marker, "yaw_velocity_ki: 0.05")
    try:
        deploy(candidate, "yaw-ki-005-selected")
    except Exception:
        deploy(old, "yaw-ki-005-rollback")
        raise
    OUTPUT.parent.mkdir(exist_ok=True)
    OUTPUT.write_text(json.dumps({
        "deployed_at_utc": datetime.now(timezone.utc).isoformat(),
        "host": "alliance-hero", "robot_library_sha256": EXPECTED_LIBRARY,
        "prior_yaml_sha256": sha(old), "deployed_yaml_sha256": sha(candidate),
        "change": {"yaw_velocity_ki": {"from": 0.02, "to": 0.05}},
        "c_car_executor_running": True,
    }, indent=2) + "\n")
    print("C-car yaw Ki=0.05 deployed; YAML SHA-256", sha(candidate), flush=True)


if __name__ == "__main__":
    main()
