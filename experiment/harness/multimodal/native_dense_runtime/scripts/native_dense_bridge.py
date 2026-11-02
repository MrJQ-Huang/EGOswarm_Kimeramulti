#!/usr/bin/env python3
"""Executable ROS package entry point; implementation stays in the harness."""
import os
from pathlib import Path
import runpy
import sys

harness = Path(os.environ['NATIVE_DENSE_HARNESS']).resolve(strict=True)
implementation = harness/'native_dense_bridge.py'
if not implementation.is_file(): raise SystemExit('Native dense harness implementation is missing')
sys.path.insert(0, str(harness))
runpy.run_path(str(implementation), run_name='__main__')
