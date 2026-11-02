#!/usr/bin/env python3
"""Verify the actual sourced overlay, native executable linkage and message ABI."""
import argparse
import hashlib
import json
import platform
from pathlib import Path
import subprocess
import time


def run(args):
    return subprocess.check_output(args, text=True, stderr=subprocess.STDOUT).strip()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--workspace", type=Path, required=True)
    p.add_argument("--underlay", type=Path, required=True)
    a = p.parse_args()
    ws = a.workspace.resolve()
    old = json.loads((ws / "evidence/underlay_abi.json").read_text())
    messages = {t: run(["rosmsg", "md5", t]) for t in old["message_md5"]}
    errors = []
    if messages != old["message_md5"]:
        errors.append("Dense overlay changed exchanged PoseGraph message MD5")
    packages = {}
    for package in ("kimera_pgmo", "kimera_pgmo_ros", "kimera_pgmo_msgs", "kimera_semantics",
                    "kimera_semantics_ros", "voxblox", "voxblox_ros", "voxblox_msgs"):
        path = Path(run(["rospack", "find", package])).resolve()
        packages[package] = str(path)
        if ws not in path.parents:
            errors.append(package + " resolves outside the new dense overlay")
    programs = {"kimera_pgmo_ros": ["kimera_pgmo_node", "voxblox_mesh_frontend_node", "mesh_frontend_node"],
                "kimera_semantics_ros": ["kimera_semantics_node"], "voxblox_ros": ["tsdf_server"]}
    executables = {}
    for package, names in programs.items():
        for name in names:
            candidates = [ws / "devel/lib" / package / name]
            candidates += list((ws / "devel/.private" / package / "lib" / package).glob(name))
            executable = next((f for f in candidates if f.is_file()), None)
            if executable is None:
                errors.append("Missing native executable: " + package + "/" + name)
                continue
            linkage = run(["ldd", str(executable)])
            executables[package + "/" + name] = {"path": str(executable), "architecture": run(["file", str(executable)]),
                "sha256": hashlib.sha256(executable.read_bytes()).hexdigest(), "ldd": linkage}
            if "not found" in linkage:
                errors.append("Missing shared library: " + package + "/" + name)
    record = {"wall_time": time.time(), "architecture": platform.machine(), "workspace": str(ws),
              "underlay": str(a.underlay), "message_md5": messages, "packages": packages,
              "executables": executables, "errors": errors,
              "status": "native_packages_and_linkage_verified" if not errors else "failed",
              "not_yet_verified": ["native TSDF input wiring", "native PGMO mesh deformation", "dual-host runtime"]}
    (ws / "evidence/overlay_verification.json").write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps({k: v for k, v in record.items() if k != "executables"}, indent=2))
    if errors:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
