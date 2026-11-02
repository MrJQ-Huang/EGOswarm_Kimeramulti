#!/usr/bin/env python3
"""Install the pinned source-only Noetic-unreleased runtime dependency."""
import argparse
import hashlib
import json
from pathlib import Path
import tarfile
import time

EXPECTED = "655817cd3002c5661bddb88423e15ec7a93d62450735a542e6dc0ad8579afb21"


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--bundle", type=Path, required=True)
    p.add_argument("--workspace", type=Path, required=True)
    a = p.parse_args()
    w = a.workspace.resolve()
    if not (w / "source_validation.json").is_file() or "native_full_stack_20261102" not in w.parts:
        raise SystemExit("Validated isolated dense workspace required")
    if hashlib.sha256(a.bundle.read_bytes()).hexdigest() != EXPECTED:
        raise SystemExit("Supplement checksum mismatch")
    with tarfile.open(a.bundle) as tf:
        for m in tf:
            out = w / m.name
            try:
                out.resolve().relative_to(w)
            except ValueError:
                raise SystemExit("Unsafe source supplement path")
            if m.isdir():
                out.mkdir(parents=True, exist_ok=True)
            elif m.isfile():
                data = tf.extractfile(m).read()
                if data.startswith(b"\x7fELF"):
                    raise SystemExit("Compiled artifact forbidden")
                if out.exists() and out.read_bytes() != data:
                    raise SystemExit("Existing source differs: " + str(out))
                if not out.exists():
                    out.parent.mkdir(parents=True, exist_ok=True)
                    out.write_bytes(data)
                    out.chmod(m.mode & 0o777)
            else:
                raise SystemExit("Unsupported supplement member")
    lock = json.loads((w / "disparity_source_metadata/source_lock_supplement.json").read_text())
    for name, expected in lock["files"].items():
        if hashlib.sha256((w / lock["directory"] / name).read_bytes()).hexdigest() != expected:
            raise SystemExit("Supplement source hash mismatch: " + name)
    receipt = {"wall_time": time.time(), "bundle_sha256": EXPECTED, "commit": lock["commit"],
               "files_verified": len(lock["files"]), "status": "validated_not_built"}
    (w / "disparity_validation.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps(receipt))


if __name__ == "__main__":
    main()
