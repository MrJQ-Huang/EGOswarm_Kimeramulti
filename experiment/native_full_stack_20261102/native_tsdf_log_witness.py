#!/usr/bin/env python3
"""Read existing native ROS logs only; completion messages follow real integration.

The pinned tsdf_server.cc logs Finished integrating only after
integratePointcloud returns (lines305-311). Mesh-timer lines are counted
separately. No ROS services, parameters or publishers are used.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import re
import signal
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--cutoff', required=True, type=float)
    parser.add_argument('--rosout-log', required=True, type=Path)
    parser.add_argument('--output-directory', required=True, type=Path)
    parser.add_argument('--duration', type=float, default=1500.)
    args = parser.parse_args()
    args.output_directory.mkdir(parents=True, exist_ok=True)
    stopped = [False]
    def stop(_sig, _frame): stopped[0] = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    started = time.monotonic()
    handle = args.rosout_log.open('r', encoding='utf-8', errors='replace')
    events = (args.output_directory/'integration_events.jsonl').open('a', buffering=1)
    samples = (args.output_directory/'integration_samples.jsonl').open('a', buffering=1)
    counts = {str(robot): Counter() for robot in (0, 1)}
    latest = {str(robot): {} for robot in (0, 1)}
    node = re.compile(r'^(\d+\.\d+)\s+\S+\s+/sim_([01])/native_tsdf\s')
    finished = re.compile(r'Finished integrating in ([\d.]+) seconds, have (\d+) blocks\.')
    points = re.compile(r'Integrating a pointcloud with (\d+) points\.')
    last_sample = 0.
    meta = dict(run_id=args.run_id, cutoff=args.cutoff, rosout_log=str(args.rosout_log),
        started_wall=time.time(), readonly=True, no_ROS_connections=True,
        interpretation='Completion line follows real integratePointcloud return; no per-cloud stamp in native log, no assertion of every input integrated')
    (args.output_directory/'integration_meta.json').write_text(json.dumps(meta, indent=2))
    while not stopped[0] and time.monotonic()-started < args.duration and not (args.output_directory/'STOP').exists():
        for line in handle:
            match = node.search(line)
            if not match or float(match[1]) < args.cutoff:
                continue
            wall = float(match[1]); robot = match[2]
            done = finished.search(line)
            begin = points.search(line)
            event = None
            if done:
                counts[robot]['actual_integration_completions'] += 1
                latest[robot].update(completion_wall=wall, integration_seconds=float(done[1]), allocated_blocks=int(done[2]))
                latest[robot]['maximum_integration_seconds'] = max(latest[robot].get('maximum_integration_seconds', 0.), float(done[1]))
                event = dict(event='actual_native_tsdf_integration_returned', robot_id=int(robot),
                    source_wall=wall, integration_seconds=float(done[1]), allocated_blocks=int(done[2]),
                    count=counts[robot]['actual_integration_completions'])
            elif begin:
                counts[robot]['integration_begin_messages'] += 1
                latest[robot]['input_points'] = int(begin[1])
            elif 'Input pointcloud queue getting too long!' in line:
                counts[robot]['throttled_queue_drop_warnings'] += 1
                event = dict(event='native_tsdf_queue_drop_warning', robot_id=int(robot), source_wall=wall, text=line.rstrip())
            elif 'Updating mesh.' in line:
                counts[robot]['mesh_timer_updates_not_integration_receipts'] += 1
            elif any(token in line.lower() for token in ('transform', 'dropping', 'failed', 'error')):
                counts[robot]['other_problem_messages'] += 1
                event = dict(event='native_tsdf_problem_log', robot_id=int(robot), source_wall=wall, text=line.rstrip())
            if event:
                events.write(json.dumps(event, allow_nan=False)+'\n')
        now = time.time()
        if now-last_sample > 1.:
            record = dict(wall=now, counts={r: dict(c) for r, c in counts.items()}, latest=latest)
            samples.write(json.dumps(record, allow_nan=False)+'\n')
            (args.output_directory/'integration_latest.json').write_text(json.dumps(record, indent=2))
            last_sample = now
        time.sleep(.2)
    final = dict(meta, ended_wall=time.time(), counts={r: dict(c) for r, c in counts.items()}, latest=latest)
    (args.output_directory/'integration_summary.json').write_text(json.dumps(final, indent=2))
    handle.close(); events.close(); samples.close()


if __name__ == '__main__':
    main()
