#!/usr/bin/env python3
"""Read-only Linux host/process/interface counters for a named native experiment."""
import argparse
from collections import defaultdict
import json
import math
import os
from pathlib import Path
import platform
import re
import signal
import socket
import statistics
import time

NATIVE_EXES = {"kimera_vio_ros_node", "kimera_distributed_loop_closure_node", "dpgo_ros_node",
               "kimera_pgmo_node", "voxblox_mesh_frontend_node", "mesh_frontend_node",
               "kimera_semantics_node", "tsdf_server", "esdf_server"}
EGO_EXES = {"ego_planner_node", "traj_server", "trajectory_server", "pcl_render_node",
            "map_generator", "map_generator_node", "grid_map_node", "odom_visualization"}
PYTHON_SCRIPTS = {"native_vio_graph_adapter.py", "native_dense_bridge.py", "native_dense_global_view.py",
                  "native_local_sensor_bridge.py", "native_round_coordinator.py",
                  "sensor_uplink.py", "sensor_ingress.py", "closed_loop_missions.py", "complex_world_view.py",
                  "continuous_body.py", "sim_sensors.py", "peer_heartbeat.py", "live_trajectory_view.py",
                  "scenario_supervisor.py", "continuous_mission.py", "broad_loop_mission.py"}
PYTHON_ROOTS = ("/experiment/harness/", "/home/amov/Drone/two_pc_ego_20261030/harness/")
PAGE_BYTES = os.sysconf("SC_PAGE_SIZE") if os.name == "posix" else 4096
TICKS = os.sysconf("SC_CLK_TCK") if os.name == "posix" else 100


def select_role(argv, include_rviz=False):
    if not argv:
        return None
    executable = Path(argv[0]).name
    if executable in NATIVE_EXES:
        return "native:" + executable
    if executable in EGO_EXES:
        return "ego:" + executable
    if include_rviz and executable == "rviz":
        return "visualization:rviz"
    # Exact script argument matching excludes docker/roslaunch wrappers which
    # merely contain a node's name elsewhere in their command line.
    if executable in ("python", "python3", "python3.8", "python3.10") and len(argv) > 1:
        script = argv[1]
        if Path(script).name in PYTHON_SCRIPTS and script.startswith(PYTHON_ROOTS):
            return "adapter_or_simulation:" + Path(script).name
    return None


def parse_stat(text):
    close = text.rfind(")")
    fields = text[close + 2:].split()
    return {"state": fields[0], "cpu_ticks": int(fields[11]) + int(fields[12]),
            "threads": int(fields[17]), "start_ticks": int(fields[19]),
            "rss_bytes": int(fields[21]) * PAGE_BYTES}


def percentile(values, q):
    values = sorted(x for x in values if x is not None and math.isfinite(x))
    if not values:
        return None
    position = (len(values) - 1) * q
    low = int(math.floor(position))
    high = int(math.ceil(position))
    return values[low] + (values[high] - values[low]) * (position - low)


def distribution(values):
    valid = [x for x in values if x is not None and math.isfinite(x)]
    return {"samples": len(valid), "min": min(valid) if valid else None,
            "p50": percentile(valid, .5), "p95": percentile(valid, .95),
            "max": max(valid) if valid else None,
            "mean": statistics.fmean(valid) if valid else None}


def memory():
    info = {}
    for line in Path("/proc/meminfo").read_text().splitlines():
        key, value = line.split(":", 1)
        info[key] = int(value.split()[0]) * 1024
    return {"ram_total_bytes": info["MemTotal"], "ram_available_bytes": info["MemAvailable"],
            "ram_unavailable_bytes": info["MemTotal"] - info["MemAvailable"],
            "swap_total_bytes": info["SwapTotal"], "swap_used_bytes": info["SwapTotal"] - info["SwapFree"]}


def cpu():
    first = Path("/proc/stat").read_text().splitlines()[0].split()[1:9]
    fields = [int(x) for x in first]
    return {"total_ticks": sum(fields), "idle_iowait_ticks": fields[3] + fields[4]}


def network():
    result = {}
    for line in Path("/proc/net/dev").read_text().splitlines()[2:]:
        interface, raw = line.split(":", 1)
        x = [int(v) for v in raw.split()]
        result[interface.strip()] = {"rx_bytes": x[0], "rx_packets": x[1], "rx_errors": x[2],
                                    "rx_drops": x[3], "tx_bytes": x[8], "tx_packets": x[9],
                                    "tx_errors": x[10], "tx_drops": x[11]}
    return result


def network_delta(current, previous, dt):
    if previous is None or dt <= 0:
        return {"rx_bytes_per_s": None, "tx_bytes_per_s": None, "counter_reset": False}
    rx, tx = current["rx_bytes"] - previous["rx_bytes"], current["tx_bytes"] - previous["tx_bytes"]
    reset = rx < 0 or tx < 0
    return {"rx_bytes_per_s": rx / dt if not reset else None,
            "tx_bytes_per_s": tx / dt if not reset else None, "counter_reset": reset}


def processes(master_uri, include_rviz):
    selected, errors = [], defaultdict(int)
    for item in Path("/proc").glob("[0-9]*/cmdline"):
        try:
            argv = [x.decode(errors="replace") for x in item.read_bytes().split(b"\0") if x]
            role = select_role(argv, include_rviz)
            if role is None:
                continue
            stat = parse_stat(item.with_name("stat").read_text())
            if stat["state"] == "Z":
                continue
            env = dict(x.split(b"=", 1) for x in item.with_name("environ").read_bytes().split(b"\0") if b"=" in x)
            uri = env.get(b"ROS_MASTER_URI", b"").decode(errors="replace").rstrip("/")
            if master_uri and uri != master_uri:
                errors["selected_name_different_master"] += 1
                continue
            selected.append(dict(stat, pid=int(item.parent.name), role=role, argv=argv, master_uri=uri))
        except (FileNotFoundError, ProcessLookupError):
            errors["process_ended_during_read"] += 1
        except PermissionError:
            errors["selected_process_permission_denied"] += 1
        except (ValueError, IndexError):
            errors["proc_metadata_parse_error"] += 1
    return selected, dict(errors)


def self_test():
    assert select_role(["docker", "exec", "x", "kimera_vio_ros_node"]) is None
    assert select_role(["/opt/ws/lib/kimera_vio_ros_node"]) == "native:kimera_vio_ros_node"
    assert select_role(["python3", "/experiment/harness/multimodal/continuous_body.py"]).startswith("adapter_")
    assert select_role(["python3", "/unrelated/continuous_body.py"]) is None
    fields = ["S"] + ["0"] * 21
    fields[11], fields[12], fields[17], fields[19], fields[21] = "120", "30", "4", "900", "17"
    result = parse_stat("123 (a process with spaces) " + " ".join(fields))
    assert result["cpu_ticks"] == 150 and result["start_ticks"] == 900 and result["rss_bytes"] == 17 * PAGE_BYTES
    assert percentile([1, 2, 3, 4], .5) == 2.5
    d = network_delta({"rx_bytes": 140, "tx_bytes": 220}, {"rx_bytes": 100, "tx_bytes": 100}, 2)
    assert d["rx_bytes_per_s"] == 20 and d["tx_bytes_per_s"] == 60
    assert network_delta({"rx_bytes": 1, "tx_bytes": 2}, {"rx_bytes": 3, "tx_bytes": 4}, 1)["counter_reset"]
    assert 100.0 * (200 / 100) / 2 == 100.0  # 200 ticks/2s at HZ100 = one full core.
    print(json.dumps({"self_test": "pass", "samples_collected": 0}))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-id")
    ap.add_argument("--host", choices=("uros", "amov", "mrjq311"))
    ap.add_argument("--output-prefix", type=Path)
    ap.add_argument("--ros-master-uri", help="Exact normalized ROS master filter for process ownership")
    ap.add_argument("--duration", type=float, default=900)
    ap.add_argument("--interval", type=float, default=1)
    ap.add_argument("--start-at", type=float, help="Optional synchronized UNIX wall start time")
    ap.add_argument("--include-rviz", action="store_true")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        self_test()
        return
    if not args.run_id or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,95}", args.run_id):
        ap.error("A unique simple --run-id is required")
    if args.host is None or args.output_prefix is None:
        ap.error("--host and --output-prefix are required")
    if not .2 <= args.interval <= 60 or not 0 < args.duration <= 86400:
        ap.error("Use interval .2..60 seconds and duration >0..86400 seconds")
    prefix = args.output_prefix.resolve()
    prefix.parent.mkdir(parents=True, exist_ok=True)
    raw_path = Path(str(prefix) + ".jsonl")
    meta_path = Path(str(prefix) + ".meta.json")
    summary_path = Path(str(prefix) + ".summary.json")
    if any(p.exists() for p in (raw_path, meta_path, summary_path)):
        raise SystemExit("Preserve existing samples; choose a fresh output prefix")
    stop = {"requested": False, "signal": None}

    def stopping(signum, _):
        stop.update(requested=True, signal=signum)

    signal.signal(signal.SIGINT, stopping)
    signal.signal(signal.SIGTERM, stopping)
    master_uri = args.ros_master_uri.rstrip("/") if args.ros_master_uri else None
    ncpu = len(os.sched_getaffinity(0))
    meta = {"run_id": args.run_id, "host": args.host, "hostname": socket.gethostname(),
            "architecture": platform.machine(), "logical_cpus_available": ncpu, "clock_ticks_per_second": TICKS,
            "page_bytes": PAGE_BYTES, "requested_duration_s": args.duration, "requested_interval_s": args.interval,
            "requested_start_wall": args.start_at, "master_uri_filter": master_uri,
            "cpu_scope": "Selected actual executable/script processes. One-core utilization uses 100% per logical CPU; normalized utilization divides by available CPU count.",
            "rss_scope": "Sum of resident set sizes, not USS/PSS; shared pages can appear in several processes.",
            "network_scope": "Whole-host per-interface counters, including SSH and unrelated workloads. Virtual interfaces may count the same packet. Interfaces are never added together and this is not SLAM-only bandwidth.",
            "native_executables": sorted(NATIVE_EXES), "ego_executables": sorted(EGO_EXES),
            "adapter_scripts": sorted(PYTHON_SCRIPTS), "script_path_roots": list(PYTHON_ROOTS),
            "include_rviz": args.include_rviz, "read_only_system_access": True}
    meta_path.write_text(json.dumps(meta, indent=2) + "\n")
    while args.start_at and time.time() < args.start_at and not stop["requested"]:
        time.sleep(min(.1, max(0, args.start_at - time.time())))
    began, began_wall = time.monotonic(), time.time()
    previous_proc, previous_net, previous_cpu, previous_mono = {}, {}, None, None
    metrics, role_metrics, net_metrics = defaultdict(list), defaultdict(lambda: defaultdict(list)), defaultdict(lambda: defaultdict(list))
    interface_first, interface_last, resets, all_processes = {}, {}, defaultdict(int), {}
    samples, read_errors = 0, defaultdict(int)
    raw = raw_path.open("x", buffering=1)
    try:
        next_tick = began
        while not stop["requested"] and time.monotonic() - began < args.duration:
            now, wall = time.monotonic(), time.time()
            dt = now - previous_mono if previous_mono is not None else None
            procs, errors = processes(master_uri, args.include_rviz)
            for k, v in errors.items():
                read_errors[k] += v
            current_proc, by_role = {}, defaultdict(lambda: {"rss_bytes": 0, "cpu_percent_one_core": 0., "processes": 0})
            selected_cpu = 0.
            cpu_known = previous_mono is not None
            for row in procs:
                identity = (row["pid"], row["start_ticks"])
                old = previous_proc.get(identity)
                value = 100. * (row["cpu_ticks"] - old["cpu_ticks"]) / TICKS / dt if old and dt else None
                row["cpu_percent_one_core"] = value
                if value is not None:
                    selected_cpu += value
                current_proc[identity] = row
                role = by_role[row["role"]]
                role["rss_bytes"] += row["rss_bytes"]
                role["cpu_percent_one_core"] += value or 0.
                role["processes"] += 1
                all_processes[str(row["pid"]) + ":" + str(row["start_ticks"])] = {
                    "pid": row["pid"], "start_ticks": row["start_ticks"], "role": row["role"],
                    "argv": row["argv"], "master_uri": row["master_uri"]}
            cpu_now, mem, net = cpu(), memory(), network()
            host_busy = None
            if previous_cpu:
                total = cpu_now["total_ticks"] - previous_cpu["total_ticks"]
                idle = cpu_now["idle_iowait_ticks"] - previous_cpu["idle_iowait_ticks"]
                host_busy = 100. * (total - idle) / total if total > 0 else None
            for interface, counters in net.items():
                interface_first.setdefault(interface, dict(counters))
                interface_last[interface] = dict(counters)
                rates = network_delta(counters, previous_net.get(interface), dt or 0.)
                counters.update(rates)
                resets[interface] += int(rates["counter_reset"])
                for key in ("rx_bytes_per_s", "tx_bytes_per_s"):
                    net_metrics[interface][key].append(rates[key])
            values = dict(mem, host_cpu_busy_percent=host_busy, selected_process_count=len(procs),
                          selected_rss_bytes=sum(x["rss_bytes"] for x in procs),
                          selected_cpu_percent_one_core=selected_cpu if cpu_known else None,
                          selected_cpu_percent_host_normalized=selected_cpu / ncpu if cpu_known else None)
            record = {"run_id": args.run_id, "host": args.host, "sample": samples, "wall_time": wall,
                      "monotonic_time": now, "elapsed_s": now - began, "actual_interval_s": dt,
                      "metrics": values, "processes": procs, "by_role": dict(by_role),
                      "interfaces": net, "proc_read_errors": errors}
            raw.write(json.dumps(record, separators=(",", ":")) + "\n")
            for key, value in values.items():
                metrics[key].append(value)
            for role, data in by_role.items():
                for key, value in data.items():
                    role_metrics[role][key].append(value)
            samples += 1
            previous_proc, previous_net, previous_cpu, previous_mono = current_proc, net, cpu_now, now
            next_tick += args.interval
            while not stop["requested"] and time.monotonic() < next_tick:
                time.sleep(max(0., min(.1, next_tick - time.monotonic())))
    finally:
        raw.close()
        network_summary = {}
        for interface, data in net_metrics.items():
            first, last = interface_first[interface], interface_last[interface]
            network_summary[interface] = {"rates": {k: distribution(v) for k, v in data.items()},
                "first_counters": first, "last_counters": last, "counter_resets": resets[interface],
                "observed_rx_bytes_delta": last["rx_bytes"] - first["rx_bytes"] if not resets[interface] else None,
                "observed_tx_bytes_delta": last["tx_bytes"] - first["tx_bytes"] if not resets[interface] else None}
        result = {"run_id": args.run_id, "host": args.host, "started_wall": began_wall, "ended_wall": time.time(),
                  "elapsed_s": time.monotonic() - began, "samples": samples, "stop_signal": stop["signal"],
                  "completed_requested_duration": not stop["requested"], "raw_jsonl": str(raw_path),
                  "metadata": str(meta_path), "metrics": {k: distribution(v) for k, v in metrics.items()},
                  "by_role_observed_samples": {r: {k: distribution(v) for k, v in d.items()} for r, d in role_metrics.items()},
                  "process_identities": list(all_processes.values()), "interfaces": network_summary,
                  "read_errors": dict(read_errors), "network_scope": meta["network_scope"], "rss_scope": meta["rss_scope"]}
        summary_path.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps({"run_id": args.run_id, "host": args.host, "samples": samples,
                          "summary": str(summary_path), "selected_process_identities": len(all_processes)}), flush=True)


if __name__ == "__main__":
    main()
