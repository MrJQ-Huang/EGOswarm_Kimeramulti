#!/usr/bin/env python3
"""ROS executable wrapper for the independent local sensor input producer."""
import os
from pathlib import Path
import runpy
import sys
harness = Path(os.environ['NATIVE_DENSE_HARNESS']).resolve(strict=True)
sys.path.insert(0, str(harness))
runpy.run_path(str(harness/'native_local_sensor_bridge.py'), run_name='__main__')
