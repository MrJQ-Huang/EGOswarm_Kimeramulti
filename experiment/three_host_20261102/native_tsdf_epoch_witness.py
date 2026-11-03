#!/usr/bin/env python3
"""Tail only this run's current-master rosout.log; no ROS topic connections."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import re
import signal
import time
from urllib.parse import urlparse
import xmlrpc.client

LINE = re.compile(r'^(\d+\.\d+)\s+\S+\s+/sim_([01])/native_tsdf\s')
DONE = re.compile(r'Finished integrating in ([\d.]+) seconds, have (\d+) blocks\.')
BEGIN = re.compile(r'Integrating a pointcloud with (\d+) points\.')
UUID = re.compile(r'[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{12}')


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + '\n')
    temporary.replace(path)


class TimedTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 2.
        return connection


def rpc(uri, method, *args):
    if method not in ('getUri', 'getParam'):
        raise ValueError('Only read-only master methods are allowed')
    with xmlrpc.client.ServerProxy(uri, transport=TimedTransport()) as proxy:
        code, message, value = getattr(proxy, method)('/three_host_tsdf_log_reader', *args)
    if code != 1:
        raise RuntimeError('Master ' + method + ' failed: ' + str(message))
    return value


def read_master_uuid(uri):
    actual = str(rpc(uri, 'getUri')).rstrip('/')
    if actual != uri.rstrip('/'):
        raise ValueError('Master identity changed: ' + actual)
    value = rpc(uri, 'getParam', '/run_id')
    if not isinstance(value, str) or not UUID.fullmatch(value):
        raise ValueError('Invalid current master UUID: ' + repr(value))
    return value


def validate_epoch(path, run_id, cutoff_ns, master_uri):
    value = json.loads(path.read_text())
    expected = dict(run_id=run_id, cutoff_ns=cutoff_ns, master_uri=master_uri)
    if any(value.get(key) != wanted for key, wanted in expected.items()):
        raise ValueError('Current run/epoch/cutoff identity changed: ' + repr(value))


def current_log_path(log_root, master_uuid):
    if not UUID.fullmatch(master_uuid):
        raise ValueError('Unsafe master UUID')
    root = log_root.resolve()
    path = root / master_uuid / 'rosout.log'
    if path.resolve().parent.parent != root or path.is_symlink() or path.parent.is_symlink():
        raise ValueError('Current rosout path escapes the owned run or uses a symlink')
    return path


class IntegrationCounts:
    def __init__(self, cutoff):
        self.cutoff = cutoff
        self.counts = {str(robot): Counter() for robot in (0, 1)}
        self.latest = {str(robot): {} for robot in (0, 1)}
        self.ignored_before_cutoff = 0

    def ingest(self, line):
        match = LINE.search(line)
        if not match:
            return None
        wall, robot = float(match[1]), match[2]
        if wall < self.cutoff:
            self.ignored_before_cutoff += 1
            return None
        done, begin = DONE.search(line), BEGIN.search(line)
        if done:
            seconds = float(done[1])
            if not math.isfinite(seconds) or seconds < 0:
                raise ValueError('Malformed native integration duration')
            self.counts[robot]['actual_integration_completions'] += 1
            latest = self.latest[robot]
            latest.update(completion_wall=wall, integration_seconds=seconds, allocated_blocks=int(done[2]))
            latest['maximum_integration_seconds'] = max(latest.get('maximum_integration_seconds', 0.), seconds)
            return dict(event='actual_native_tsdf_integration_returned', robot_id=int(robot),
                        source_wall=wall, integration_seconds=seconds, allocated_blocks=int(done[2]),
                        count=self.counts[robot]['actual_integration_completions'])
        if begin:
            self.counts[robot]['integration_begin_messages'] += 1
            self.latest[robot]['input_points'] = int(begin[1])
        elif 'Input pointcloud queue getting too long!' in line:
            self.counts[robot]['throttled_queue_drop_warnings'] += 1
            return dict(event='native_tsdf_queue_drop_warning', robot_id=int(robot), source_wall=wall, text=line.rstrip())
        elif 'Updating mesh.' in line:
            self.counts[robot]['mesh_timer_updates_not_integration_receipts'] += 1
        elif any(token in line.lower() for token in ('transform', 'dropping', 'failed', 'error')):
            self.counts[robot]['other_problem_messages'] += 1
            return dict(event='native_tsdf_problem_log', robot_id=int(robot), source_wall=wall, text=line.rstrip())
        return None

    def sample(self, wall):
        return dict(wall=wall, counts={robot: dict(count) for robot, count in self.counts.items()},
                    latest=self.latest, ignored_before_cutoff=self.ignored_before_cutoff,
                    completion_ages_s={robot: wall-value['completion_wall'] if 'completion_wall' in value else None
                                       for robot, value in self.latest.items()})


def run(args):
    output = args.output_directory
    output.mkdir(parents=True, exist_ok=True)
    stopped = [False]
    for kind in (signal.SIGINT, signal.SIGTERM):
        signal.signal(kind, lambda *_: stopped.__setitem__(0, True))
    started = time.monotonic()
    cutoff_ns = round(args.cutoff * 1e9)
    counts = IntegrationCounts(args.cutoff)
    meta = dict(run_id=args.run_id, cutoff_ns=cutoff_ns, master_uri=args.master_uri,
                ros_log_root=str(args.ros_log_root), epoch_file=str(args.epoch_file),
                started_wall=time.time(), read_only=True, no_ROS_node_or_topic_connections=True,
                interpretation='Completion log follows the native integratePointcloud return; no per-cloud timestamp or every-input-integrated claim')
    fatal = None
    handle = None
    master_uuid = None
    log_identity = None
    pending = ''
    last_guard = -math.inf
    last_sample = -math.inf
    events = (output / 'integration_events.jsonl').open('a', buffering=1)
    samples = (output / 'integration_samples.jsonl').open('a', buffering=1)
    try:
        validate_epoch(args.epoch_file, args.run_id, cutoff_ns, args.master_uri)
        while not stopped[0] and time.monotonic() - started < args.duration:
            now = time.monotonic()
            if now-last_guard >= 2.:
                validate_epoch(args.epoch_file, args.run_id, cutoff_ns, args.master_uri)
                try:
                    current = read_master_uuid(args.master_uri)
                except Exception:
                    if master_uuid is not None or now-started >= args.wait_timeout:
                        raise
                    current = None
                if master_uuid is not None and current != master_uuid:
                    raise ValueError('Master UUID changed during the run')
                if current is not None:
                    master_uuid = current
                last_guard = now
            if master_uuid is not None and handle is None:
                path = current_log_path(args.ros_log_root, master_uuid)
                if path.is_file():
                    handle = path.open('r', encoding='utf-8', errors='replace')
                    stat = path.stat()
                    log_identity = (stat.st_dev, stat.st_ino)
                    meta.update(master_uuid=master_uuid, rosout_log=str(path),
                                source_inode=list(log_identity), opened_wall=time.time())
                    write_json(output / 'integration_meta.json', meta)
            if handle is None:
                if now-started >= args.wait_timeout:
                    raise RuntimeError('Current run master/rosout.log did not become ready before the deadline')
            else:
                stat = path.stat()
                if (stat.st_dev, stat.st_ino) != log_identity or stat.st_size < handle.tell():
                    raise ValueError('Current native rosout.log was replaced or truncated')
                pending += handle.read()
                lines = pending.split('\n')
                pending = lines.pop()
                for line in lines:
                    event = counts.ingest(line)
                    if event:
                        events.write(json.dumps(event, allow_nan=False) + '\n')
            if now-last_sample >= 1.:
                record = dict(counts.sample(time.time()), run_id=args.run_id, cutoff_ns=cutoff_ns,
                              master_uuid=master_uuid, waiting_for_current_log=handle is None, fatal=None)
                samples.write(json.dumps(record, allow_nan=False) + '\n')
                write_json(output / 'integration_latest.json', record)
                last_sample = now
            time.sleep(.2)
    except Exception as exc:
        fatal = type(exc).__name__ + ': ' + str(exc)
    finally:
        if handle is None and fatal is None:
            fatal = 'Current run native rosout.log was never opened'
        final = dict(meta, ended_wall=time.time(), fatal=fatal, **counts.sample(time.time()))
        write_json(output / 'integration_summary.json', final)
        write_json(output / 'integration_latest.json', final)
        if handle:
            handle.close()
        events.close()
        samples.close()
    return 2 if fatal else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--cutoff', required=True, type=float)
    parser.add_argument('--master-uri', default='http://192.168.5.14:11431')
    parser.add_argument('--epoch-file', type=Path, required=True)
    parser.add_argument('--ros-log-root', type=Path, required=True)
    parser.add_argument('--output-directory', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=1500.)
    parser.add_argument('--wait-timeout', type=float, default=60.)
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,95}', args.run_id):
        parser.error('Invalid run ID')
    for value in (args.cutoff, args.duration, args.wait_timeout):
        if not math.isfinite(value) or value <= 0:
            parser.error('Cutoff, duration and wait timeout must be finite and positive')
    uri = urlparse(args.master_uri)
    if (uri.scheme != 'http' or (uri.hostname, uri.port) not in (('192.168.5.14',11431),('127.0.0.1',11561))
            or uri.username or uri.password or uri.query or uri.fragment or uri.path not in ('','/')):
        parser.error('Use the dedicated hub master or isolated loopback fixture master')
    args.master_uri = args.master_uri.rstrip('/')
    raise SystemExit(run(args))


if __name__ == '__main__':
    main()
