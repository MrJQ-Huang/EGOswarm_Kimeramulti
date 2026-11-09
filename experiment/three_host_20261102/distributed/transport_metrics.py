#!/usr/bin/env python3
"""Read Linux TCP counters for the peer and observer connections only.

RTT is the kernel's smoothed round trip estimate, never a one-way latency.
Byte deltas cover sampled live connections; a connection that disappears between
samples can leave an unobserved tail. No raw SLAM payload is collected.
"""
import argparse
import json
from pathlib import Path
import re
import subprocess
import time

PORTS = {11640: 'peer', 11740: 'map', 11741: 'telemetry'}


def parse_ss(output):
    rows = []; current = None
    for line in output.splitlines():
        parts = line.split()
        # A non-established socket record starts a new block too. Its counters
        # must never overwrite those of the previous selected live connection.
        if line and not line[0].isspace(): current = None
        if parts and parts[0] == 'ESTAB' and len(parts) >= 5:
            local, remote = parts[3], parts[4]
            try: ports = {int(local.rsplit(':', 1)[1]), int(remote.rsplit(':', 1)[1])}
            except ValueError: current = None; continue
            selected = ports & set(PORTS)
            current = None
            if len(selected) == 1:
                current = dict(local=local, remote=remote, channel=PORTS[selected.pop()])
                rows.append(current)
        elif current is not None and line[:1].isspace():
            for field in ('bytes_sent', 'bytes_received', 'bytes_retrans', 'bytes_acked'):
                match = re.search(r'\b' + field + r':(\d+)\b', line)
                if match: current[field] = int(match.group(1))
            match = re.search(r'\brtt:([\d.]+)/([\d.]+)', line)
            if match: current['smoothed_rtt_ms'] = float(match.group(1))
            match = re.search(r'\blastsnd:(\d+)\b', line)
            if match: current['last_payload_send_age_ms'] = int(match.group(1))
    return rows


def distribution(values):
    if not values: return None
    values = sorted(values)
    return dict(samples=len(values), mean=sum(values)/len(values),
                p50=values[round((len(values)-1)*.5)],
                p95=values[round((len(values)-1)*.95)], maximum=values[-1])


def summarize(samples):
    channels = {}
    previous = {}
    for sample in samples:
        seen = set()
        for row in sample['connections']:
            key = (row['local'], row['remote']); seen.add(key)
            stat = channels.setdefault(row['channel'], dict(observed_rx_bytes=0,
                observed_tx_bytes=0, observed_retransmitted_bytes=0, counter_resets=0,
                rtt_ms=[], recent_sender_rtt_ms=[], observed_endpoint_pairs=set()))
            stat['observed_endpoint_pairs'].add(key)
            if 'smoothed_rtt_ms' in row: stat['rtt_ms'].append(row['smoothed_rtt_ms'])
            if ('smoothed_rtt_ms' in row and row.get('last_payload_send_age_ms', 5001) <= 5000):
                stat['recent_sender_rtt_ms'].append(row['smoothed_rtt_ms'])
            old = previous.get(key)
            if old:
                for field, target in [('bytes_received','observed_rx_bytes'),
                                      ('bytes_sent','observed_tx_bytes'),
                                      ('bytes_retrans','observed_retransmitted_bytes')]:
                    # Linux omits a zero retransmission counter.
                    value, before = row.get(field, 0), old.get(field, 0)
                    if value >= before: stat[target] += value-before
                    else: stat['counter_resets'] += 1
            previous[key] = row
        previous = {key:value for key,value in previous.items() if key in seen}
    elapsed = samples[-1]['monotonic']-samples[0]['monotonic'] if len(samples)>1 else 0.
    for stat in channels.values():
        stat['smoothed_rtt_ms'] = distribution(stat.pop('rtt_ms'))
        stat['recent_payload_sender_rtt_ms'] = distribution(stat.pop('recent_sender_rtt_ms'))
        stat['observed_connections'] = len(stat.pop('observed_endpoint_pairs'))
        stat['observed_rx_mbit_per_s'] = stat['observed_rx_bytes']*8/elapsed/1e6 if elapsed else None
        stat['observed_tx_mbit_per_s'] = stat['observed_tx_bytes']*8/elapsed/1e6 if elapsed else None
    return dict(elapsed_s=elapsed, channels=channels,
                byte_scope='TCP payload counters on sampled live connections; no IP/Ethernet overhead; disappearing connection tails may be unobserved',
                latency_scope='kernel smoothed TCP RTT, not one-way latency or independent ping; receiver-only RTT can remain cached since handshake; recent_payload_sender_rtt_ms includes only sockets with payload sent within 5 seconds')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--duration', type=float, default=60.)
    p.add_argument('--interval', type=float, default=1.)
    p.add_argument('--run-id', required=True); p.add_argument('--host', required=True)
    p.add_argument('--output', type=Path, required=True); a = p.parse_args()
    if not 0 < a.interval <= a.duration <= 3600: p.error('Require 0 < interval <= duration <= 3600')
    samples = []; began = time.monotonic()
    while True:
        samples.append(dict(wall=time.time(), monotonic=time.monotonic(), connections=parse_ss(
            subprocess.check_output(['ss','-tin'], text=True, timeout=5))))
        remaining = began+a.duration-time.monotonic()
        if remaining <= 0: break
        time.sleep(min(a.interval, remaining))
    report = dict(run_id=a.run_id, host=a.host, **summarize(samples), samples=samples)
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k != 'samples'}))


if __name__ == '__main__': main()
