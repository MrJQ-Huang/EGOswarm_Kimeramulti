#!/usr/bin/env python3
"""Validate and unpack the frozen source-only closure into a new dense overlay."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import tarfile
import time

BUNDLE_SHA256 = "5c59408b70a89a7f1a01267b4b3ad3e5ddec2bdd1dee655ab72ec03095949bc7"


def sha256(path):
    if path.is_symlink():
        return hashlib.sha256(("SYMLINK:" + os.readlink(str(path))).encode()).hexdigest()
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def beneath(path, root):
    try:
        path.resolve().relative_to(root.resolve())
        return True
    except ValueError:
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bundle", type=Path, required=True)
    ap.add_argument("--workspace", type=Path, required=True)
    args = ap.parse_args()
    ws = args.workspace.resolve()
    if "native_full_stack_20261102" not in ws.parts or ws.name not in ("uros_ws", "amov_ws"):
        raise SystemExit("Workspace must be a new native_full_stack_20261102/{uros_ws,amov_ws} tree")
    if sha256(args.bundle) != BUNDLE_SHA256:
        raise SystemExit("Frozen source bundle checksum mismatch")
    ws.mkdir(parents=True, exist_ok=True)
    with tarfile.open(args.bundle, "r:gz") as tf:
        for member in tf.getmembers():
            out = ws / member.name
            if not beneath(out, ws):
                raise SystemExit("Unsafe archive destination: " + member.name)
            if member.islnk():
                raise SystemExit("Hard links are not part of the source bundle contract")
            if member.issym() and not beneath(out.parent / member.linkname, ws):
                raise SystemExit("Unsafe archive symlink: " + member.name)
            if not (member.isdir() or member.isfile() or member.issym()):
                raise SystemExit("Unsupported archive member: " + member.name)
        # This bundle is authenticated above. Do not overwrite edited source files.
        for member in tf.getmembers():
            out = ws / member.name
            if member.isdir():
                out.mkdir(parents=True, exist_ok=True)
            elif member.issym():
                if out.is_symlink():
                    if os.readlink(str(out)) != member.linkname:
                        raise SystemExit("Existing symlink differs: " + str(out))
                elif out.exists():
                    raise SystemExit("Existing path blocks source symlink: " + str(out))
                else:
                    out.parent.mkdir(parents=True, exist_ok=True)
                    out.symlink_to(member.linkname)
            else:
                content = tf.extractfile(member).read()
                if content.startswith(b"\x7fELF"):
                    raise SystemExit("Compiled ELF is forbidden in source bundle: " + member.name)
                if out.exists():
                    if out.read_bytes() != content:
                        raise SystemExit("Existing source differs; preserve it and choose a clean workspace: " + str(out))
                else:
                    out.parent.mkdir(parents=True, exist_ok=True)
                    out.write_bytes(content)
                    out.chmod(member.mode & 0o777)
    lock = json.loads((ws / "source_lock.json").read_text())
    checked = 0
    for repo in lock["repos"]:
        repo_root = ws / repo["directory"]
        index = json.loads((ws / "source_metadata" / (repo_root.name + ".files.json")).read_text())
        for relative, expected in index.items():
            if sha256(repo_root / relative) != expected:
                raise SystemExit("Extracted source checksum mismatch: " + str(repo_root / relative))
            checked += 1
        index_digest = hashlib.sha256("".join(k + "\0" + index[k] + "\n" for k in sorted(index)).encode()).hexdigest()
        if index_digest != repo["sha256_path_content_index"]:
            raise SystemExit("Source path/content index checksum mismatch: " + str(repo_root))
        patch = ws / "source_metadata" / (repo_root.name + ".diff")
        if sha256(patch) != repo["diff_sha256"]:
            raise SystemExit("Recorded patch checksum mismatch: " + str(patch))
    receipt = {"wall_time": time.time(), "workspace": str(ws), "bundle_sha256": BUNDLE_SHA256,
               "repositories": len(lock["repos"]), "files_verified": checked,
               "source_only": True, "status": "validated_not_built"}
    (ws / "source_validation.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps(receipt))


if __name__ == "__main__":
    main()
