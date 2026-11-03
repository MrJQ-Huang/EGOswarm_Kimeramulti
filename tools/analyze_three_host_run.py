#!/usr/bin/env python3
"""Summarize downloaded two-drone evidence without ROS or estimator inputs.

Run with --run-directory <runs/RID>; edge downloads belong in amov/ and uros/.
The actual-body trajectory is an evaluation reference only. This script never
fits, transforms or modifies any SLAM trajectory, mesh or optimizer evidence.
"""
import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import time


HOSTS = ("amov", "uros", "mrjq311")
ROBOTS = {0: "amov", 1: "uros"}
COLORS = {"amov": "#20a6bc", "uros": "#e48922", "mrjq311": "#6655aa"}
NETWORK_COUNTERS = ("rx_bytes", "tx_bytes", "rx_packets", "tx_packets",
                    "rx_errors", "tx_errors", "rx_drops", "tx_drops")


def finite(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def distribution(values):
    values = sorted(float(v) for v in values if finite(v))
    def percentile(q):
        position = (len(values) - 1) * q
        low, high = math.floor(position), math.ceil(position)
        return values[low] + (values[high] - values[low]) * (position - low)
    return {"samples": len(values), "min": values[0] if values else None,
            "p50": percentile(.5) if values else None,
            "p95": percentile(.95) if values else None,
            "max": values[-1] if values else None,
            "mean": statistics.fmean(values) if values else None}


def read_json(path, warnings):
    if not path.is_file():
        return None
    # Some live writers truncate before rewriting. A bounded retry permits a
    # partial analysis without treating a momentarily empty file as valid JSON.
    for attempt in range(3):
        try:
            value = json.loads(path.read_text())
            if not isinstance(value, dict):
                raise ValueError("Expected a JSON object")
            return value
        except (OSError, ValueError) as error:
            if attempt == 2:
                warnings.append(str(path) + ": " + str(error))
            else:
                time.sleep(.05)
    return None


def read_jsonl(path, warnings, expected_run=None):
    if not path.is_file():
        return []
    rows, invalid, foreign = [], 0, 0
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            if not line.strip():
                continue
            try:
                row = json.loads(line)
                if not isinstance(row, dict):
                    raise ValueError("Expected an object")
                if expected_run and row.get("run_id", expected_run) != expected_run:
                    foreign += 1
                    continue
                rows.append(row)
            except ValueError:
                invalid += 1
    if invalid:
        warnings.append("%s: ignored %d malformed/incomplete JSONL rows" % (path, invalid))
    if foreign:
        warnings.append("%s: rejected %d rows from another run" % (path, foreign))
    return rows


def identity_matches(value, run_id):
    return value.get("run_id") in (run_id, run_id + "_mission")


def read_mission(root, warnings):
    run_id = root.name
    candidates = (root / "uros" / (run_id + "_mission.json"),
                  root / "mrjq311/mission_live.json")
    for path in candidates:
        value = read_json(path, warnings)
        if value is not None:
            if identity_matches(value, run_id):
                return value, path
            warnings.append(str(path) + ": mission run identity mismatch")
    return None, None


def flight_window(mission, epoch):
    start = mission.get("started_wall_time") if mission else None
    end = mission.get("wall_time") if mission else None
    if finite(start) and finite(end) and end >= start:
        return {"start_wall": start, "end_wall": end, "duration_s": end - start,
                "basis": "mission.started_wall_time through mission.wall_time; includes controller initialization and waypoint dwells",
                "end_is_final": mission.get("final") is True}
    cutoff = epoch.get("cutoff_ns") if epoch else None
    start = cutoff / 1e9 if finite(cutoff) else None
    return {"start_wall": start, "end_wall": time.time(), "duration_s": None,
            "basis": "run epoch to analysis time; mission flight window unavailable",
            "end_is_final": False}


def in_window(stamp, window):
    return (finite(stamp) and (window["start_wall"] is None or stamp >= window["start_wall"])
            and stamp <= window["end_wall"])


def mission_summary(mission, path, window):
    if mission is None:
        return {"available": False, "success": None, "phase": None}
    laps = mission.get("completed_laps") or []
    stages = mission.get("completed_stages") or []
    requested = mission.get("laps_requested")
    final = mission.get("final") is True
    success = (mission.get("phase") == "closed_loops_complete_hovering"
               and not mission.get("error") and not mission.get("failure_latch")
               and isinstance(requested, int) and len(laps) >= requested) if final else None
    return {"available": True, "source": str(path), "source_is_live_fallback": path.name == "mission_live.json",
            "phase": mission.get("phase"), "final": final, "success": success,
            "duration_s": window["duration_s"], "current_lap": mission.get("lap"),
            "current_stage": mission.get("stage"), "laps_requested": requested,
            "completed_lap_count": len(laps), "completed_stage_count": len(stages),
            "completed_stage_counts_by_lap": dict(Counter(str(v.get("lap")) for v in stages)),
            "completed_stages": stages, "completed_laps": laps,
            "actual_path_length_m": mission.get("actual_path_length_m"),
            "minimum_sampled_center_separation_m": mission.get("minimum_sampled_center_separation_m"),
            "closure_error_from_recorded_start_m": mission.get("closure_error_from_recorded_start_m"),
            "error": mission.get("error"), "failure_latch": mission.get("failure_latch"),
            "interpretation": "Actual simulated body telemetry for evaluation; mission success is not SLAM geometry acceptance"}


def read_trajectory(root, mission, window, warnings):
    candidates = [root / "uros" / (root.name + "_mission.trajectory.csv")]
    if mission and isinstance(mission.get("trajectory_csv"), str):
        name = Path(mission["trajectory_csv"]).name
        candidates.extend(root / host / name for host in HOSTS)
    path = next((p for p in candidates if p.is_file()), None)
    samples = {robot: [] for robot in ROBOTS}
    if path is None:
        return samples, {"available": False, "source": None, "rows": 0}
    invalid = 0
    with path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            try:
                stamp = float(row["wall_time"])
                robot = int(row["robot_id"])
                xyz = tuple(float(row[k]) for k in ("x", "y", "z"))
                if robot not in samples or not all(finite(x) for x in xyz) or not finite(stamp):
                    raise ValueError("Invalid trajectory sample")
                if in_window(stamp, window):
                    samples[robot].append((stamp, *xyz))
            except (KeyError, ValueError, TypeError):
                invalid += 1
    if invalid:
        warnings.append("%s: ignored %d malformed CSV samples" % (path, invalid))
    details = {}
    for robot, rows in samples.items():
        rows.sort()
        distance = sum(math.dist(left[1:], right[1:]) for left, right in zip(rows, rows[1:]))
        details[ROBOTS[robot]] = {"samples": len(rows), "sampled_path_length_m": distance if rows else None,
                                 "first_wall": rows[0][0] if rows else None,
                                 "last_wall": rows[-1][0] if rows else None}
    return samples, {"available": True, "source": str(path), "rows": sum(map(len, samples.values())),
                     "per_robot": details,
                     "interpretation": "Sampled actual-body CSV path length may be below the higher-rate mission path counter; no SLAM alignment is performed"}


def integration_summary(root, window, warnings):
    folder = root / "mrjq311/tsdf_witness"
    path = folder / "integration_events.jsonl"
    meta = read_json(folder / "integration_meta.json", warnings)
    if meta and not identity_matches(meta, root.name):
        warnings.append(str(folder) + ": integration witness belongs to another run")
        return {}, {"available": False, "reason": "witness run identity mismatch"}
    events = read_jsonl(path, warnings, root.name)
    stamps = {robot: set() for robot in ROBOTS}
    for row in events:
        if row.get("event") != "actual_native_tsdf_integration_returned":
            continue
        robot, stamp = row.get("robot_id"), row.get("source_wall")
        if robot in stamps and in_window(stamp, window):
            stamps[robot].add(float(stamp))
    timelines, details = {}, {}
    for robot in ROBOTS:
        values = sorted(stamps[robot]);timelines[robot] = values
        gaps = [right - left for left, right in zip(values, values[1:])]
        start, end = window["start_wall"], window["end_wall"]
        initial = values[0] - start if values and start is not None else None
        tail = end - values[-1] if values else (end - start if start is not None else None)
        details[ROBOTS[robot]] = {"actual_integration_completions": len(values),
                                 "first_completion_wall": values[0] if values else None,
                                 "last_completion_wall": values[-1] if values else None,
                                 "maximum_between_completion_gap_s": max(gaps) if gaps else None,
                                 "initial_wait_to_first_completion_s": initial,
                                 "tail_staleness_s": tail,
                                 "maximum_gap_including_window_boundaries_s": max(gaps + [v for v in (initial, tail) if finite(v)], default=None)}
    return timelines, {"available": path.is_file(), "source": str(path), "per_robot": details,
                       "interpretation": "Counts native integratePointcloud return log events within the declared window; mesh timers and begin logs are excluded. Logs do not prove each input was integrated."}


def interface_totals(rows):
    result = {}
    names = sorted({name for row in rows for name in row.get("interfaces", {})})
    for name in names:
        values = [(row["wall_time"], row["interfaces"][name]) for row in rows if name in row.get("interfaces", {})]
        totals = {key: 0 for key in NETWORK_COUNTERS}
        known = {key: 0 for key in NETWORK_COUNTERS}
        resets, covered = 0, 0.
        for (before_wall, before), (after_wall, after) in zip(values, values[1:]):
            interval = after_wall - before_wall
            if interval <= 0:
                continue
            if any(finite(after.get(key)) and finite(before.get(key)) and after[key] < before[key]
                   for key in ("rx_bytes", "tx_bytes")):
                resets += 1
                continue
            covered += interval
            for key in NETWORK_COUNTERS:
                if finite(after.get(key)) and finite(before.get(key)) and after[key] >= before[key]:
                    totals[key] += after[key] - before[key]
                    known[key] += 1
        result[name] = {"observed_samples": len(values), "valid_interval_seconds": covered,
                        "counter_resets_skipped": resets,
                        "counter_increments": {k: totals[k] if known[k] else None for k in totals},
                        "mean_rx_bytes_per_s": totals["rx_bytes"] / covered if covered and known["rx_bytes"] else None,
                        "mean_tx_bytes_per_s": totals["tx_bytes"] / covered if covered and known["tx_bytes"] else None}
    return result


def resource_summary(root, window, warnings):
    details, plotted = {}, {}
    for host in HOSTS:
        path = root / host / "resources.jsonl"
        rows = read_jsonl(path, warnings, root.name)
        # A mislabeled downloaded host file must not be silently accepted.
        rows = [row for row in rows if row.get("host", host) == host]
        rows = sorted((row for row in rows if in_window(row.get("wall_time"), window)),
                      key=lambda row: row["wall_time"])
        plotted[host] = rows
        start, end = window["start_wall"], window["end_wall"]
        details[host] = {"available": path.is_file(), "source": str(path), "flight_window_samples": len(rows),
                         "first_sample_wall": rows[0]["wall_time"] if rows else None,
                         "last_sample_wall": rows[-1]["wall_time"] if rows else None,
                         "window_start_uncovered_s": rows[0]["wall_time"] - start if rows and start is not None else None,
                         "window_end_uncovered_s": end - rows[-1]["wall_time"] if rows else None,
                         "maximum_observed_sample_gap_s": max((b["wall_time"] - a["wall_time"] for a, b in zip(rows, rows[1:])), default=None),
                         "actual_sample_interval_s": distribution(row.get("actual_interval_s") for row in rows),
                         "host_cpu_busy_percent": distribution(row.get("metrics", {}).get("host_cpu_busy_percent") for row in rows),
                         "ram_available_bytes": distribution(row.get("metrics", {}).get("ram_available_bytes") for row in rows),
                         "interfaces": interface_totals(rows)}
    return plotted, {"hosts": details,
                      "cpu_scope": "Whole-host metrics.host_cpu_busy_percent from /proc/stat; selected-process CPU is not used",
                      "ram_scope": "Whole-host metrics.ram_available_bytes from MemAvailable; not the sum of process RSS",
                      "network_scope": "Whole-host per-interface cumulative counter increments within the window; includes SSH and unrelated traffic. Interfaces are reported separately and never summed; reset intervals are excluded."}


def global_summary(root, warnings):
    folder = root / "mrjq311/global_display"
    status = read_json(folder / "status.json", warnings)
    if status and not identity_matches(status, root.name):
        warnings.append(str(folder / "status.json") + ": run identity mismatch")
        status = None
    snapshots = []
    for path in sorted(folder.glob("native_pair_*.json")):
        row = read_json(path, warnings)
        if row is None:
            continue
        if not identity_matches(row, root.name):
            warnings.append(str(path) + ": snapshot run identity mismatch")
            continue
        if row.get("fusion_ready") is True and finite(row.get("points")) and row["points"] > 0:
            snapshots.append({"file": str(path), "points": row["points"],
                              "robot_point_counts": row.get("robot_point_counts"),
                              "cloud_stamp_ns": row.get("cloud_stamp_ns"),
                              "snapshot_identity": row.get("snapshot_identity"),
                              "binding": row.get("binding"), "completed_round_serial": row.get("completed_round_serial"),
                              "final_native_weight_batch_sha256": row.get("final_native_weight_batch_sha256")})
    identities = {row["snapshot_identity"] or row["file"] for row in snapshots}
    return {"latest_status_available": status is not None,
            "latest_fusion_ready": status.get("fusion_ready") if status else None,
            "latest_points": status.get("points") if status else None,
            "latest_reason": status.get("reason") if status else None,
            "latest_status_wall": status.get("wall") if status else None,
            "accepted_snapshot_files": len(snapshots), "unique_snapshot_identities": len(identities),
            "snapshot_scope": "All saved snapshots from this run, including warmup and drain; TSDF/resource statistics separately use the flight report window",
            "point_counts": distribution(row["points"] for row in snapshots), "snapshots": snapshots,
            "binding": status.get("binding") if status else "matched_native_observations_no_shared_atomic_round_id",
            "atomic_distributed_round_proven": False, "geometry_acceptance_proven": False,
            "interpretation": "Counts saved snapshots whose pipeline metadata says fusion_ready. Baseline evidence matches native observations without a shared atomic round ID. Metadata is not an independent geometry-quality validation; archived snapshots can exist while the latest display gate is closed."}


def write_plots(output, summary, trajectories, integrations, resources):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    suffix = " [PARTIAL]" if summary["status"] == "partial" else ""
    start = summary["flight_window"]["start_wall"]
    available_stamps = [row[0] for values in trajectories.values() for row in values]
    available_stamps += [row["wall_time"] for values in resources.values() for row in values]
    origin = start if start is not None else min(available_stamps, default=0.)
    files = []

    fig, ax = plt.subplots(figsize=(9, 6))
    for robot, rows in trajectories.items():
        if rows:
            host = ROBOTS[robot]
            ax.plot([row[1] for row in rows], [row[2] for row in rows], color=COLORS[host], label=host.upper())
            ax.scatter(rows[0][1], rows[0][2], color=COLORS[host], marker="o", s=30)
            ax.scatter(rows[-1][1], rows[-1][2], color=COLORS[host], marker="x", s=35)
    if not any(trajectories.values()):
        ax.text(.5, .5, "Actual-body trajectory CSV not downloaded\nNo path inferred from SLAM or mission endpoint", ha="center", va="center", transform=ax.transAxes)
    else:
        ax.legend()
    ax.set(title="Actual simulated body XY trajectories" + suffix, xlabel="World X (m)", ylabel="World Y (m)")
    ax.set_aspect("equal", adjustable="datalim");ax.grid(alpha=.25)
    fig.tight_layout();fig.savefig(output / "actual_xy.png", dpi=150);plt.close(fig);files.append("actual_xy.png")

    fig, ax = plt.subplots(figsize=(9, 4.5))
    for robot in ROBOTS:
        stamps = integrations.get(robot, [])
        if stamps:
            host = ROBOTS[robot]
            x = [0.] + [stamp - origin for stamp in stamps]
            y = [0] + list(range(1, len(stamps) + 1))
            if summary["flight_window"]["end_wall"] >= stamps[-1]:
                x.append(summary["flight_window"]["end_wall"] - origin);y.append(len(stamps))
            ax.step(x, y, where="post", color=COLORS[host], label=host.upper())
    if not any(integrations.values()):
        ax.text(.5, .5, "No native integration-return events available in window", ha="center", transform=ax.transAxes)
    else:
        ax.legend()
    ax.set(title="Native TSDF integration completions" + suffix, xlabel="Seconds from analysis window start", ylabel="Cumulative completed integrations")
    ax.grid(alpha=.25);fig.tight_layout();fig.savefig(output / "tsdf_integrations.png", dpi=150);plt.close(fig);files.append("tsdf_integrations.png")

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for host, rows in resources.items():
        cpu = [(row["wall_time"] - origin, row.get("metrics", {}).get("host_cpu_busy_percent")) for row in rows]
        cpu = [(x, y) for x, y in cpu if finite(y)]
        ram = [(row["wall_time"] - origin, row.get("metrics", {}).get("ram_available_bytes")) for row in rows]
        ram = [(x, y / 1024**3) for x, y in ram if finite(y)]
        if cpu:
            axes[0].plot(*zip(*cpu), color=COLORS[host], label=host.upper(), linewidth=1.)
        if ram:
            axes[1].plot(*zip(*ram), color=COLORS[host], label=host.upper(), linewidth=1.)
    for ax in axes:
        ax.grid(alpha=.25)
        if ax.lines:
            ax.legend()
        else:
            ax.text(.5, .5, "Resource samples unavailable", ha="center", transform=ax.transAxes)
    axes[0].set(title="Whole-host resource use" + suffix, ylabel="Whole-host busy CPU (%)", ylim=(0, 100))
    axes[1].set(xlabel="Seconds from analysis window start", ylabel="Available host RAM (GiB)")
    fig.tight_layout();fig.savefig(output / "host_resources.png", dpi=150);plt.close(fig);files.append("host_resources.png")
    return files


def analyze(root, output):
    warnings = []
    mission, mission_path = read_mission(root, warnings)
    epoch = read_json(root / "mrjq311/epoch_config.json", warnings)
    window = flight_window(mission, epoch)
    trajectories, trajectory = read_trajectory(root, mission, window, warnings)
    integrations, tsdf = integration_summary(root, window, warnings)
    resource_rows, resources = resource_summary(root, window, warnings)
    global_maps = global_summary(root, warnings)
    partial = []
    if not mission or mission.get("final") is not True:
        partial.append("Mission has no final report; available window is partial")
    if not trajectory["available"] or any(not trajectories[r] for r in ROBOTS):
        partial.append("Actual-body CSV samples for both robots are not available")
    if not tsdf["available"]:
        partial.append("Native TSDF completion witness is unavailable")
    for host, detail in resources["hosts"].items():
        if not detail["flight_window_samples"]:
            partial.append(host + " resource samples for this window are not available")
        elif any(finite(detail[key]) and detail[key] > 5.
                 for key in ("window_start_uncovered_s", "window_end_uncovered_s", "maximum_observed_sample_gap_s")):
            partial.append(host + " resource capture has an uncovered span over five seconds in this window")
    if not global_maps["latest_status_available"]:
        partial.append("Latest global display status is unavailable")
    if warnings:
        partial.append("Some input records were malformed, incomplete or rejected; see warnings")
    summary = {"format_version": 1, "run_id": root.name, "run_directory": str(root),
               "analyzed_at_utc": datetime.now(timezone.utc).isoformat(),
               "status": "partial" if partial else "complete", "partial_reasons": partial,
               "flight_window": window, "mission": mission_summary(mission, mission_path, window),
               "trajectory": trajectory, "tsdf": tsdf, "resources": resources,
               "global_display": global_maps, "warnings": warnings,
               "limitations": ["Actual simulated-body telemetry is used only for evaluation; no SLAM truth fitting or map alignment occurs",
                               "Neither mission completion nor native snapshot metadata proves geometric accuracy or atomic distributed fusion",
                               "All measurements use the declared controller report window; missing downloads and live inputs remain explicit"],
               "fields_used": {"mission": ["started_wall_time", "wall_time", "phase", "final", "laps_requested", "actual_path_length_m", "minimum_sampled_center_separation_m", "completed_stages", "completed_laps", "error", "failure_latch"],
                               "trajectory_csv": ["wall_time", "robot_id", "x", "y", "z"],
                               "tsdf_jsonl": ["event=actual_native_tsdf_integration_returned", "robot_id", "source_wall"],
                               "resources_jsonl": ["wall_time", "actual_interval_s", "metrics.host_cpu_busy_percent", "metrics.ram_available_bytes", "interfaces.<name>.rx_bytes/tx_bytes/rx_packets/tx_packets/rx_errors/tx_errors/rx_drops/tx_drops"],
                               "global_snapshot": ["run_id", "fusion_ready", "points", "robot_point_counts", "snapshot_identity", "binding", "cloud_stamp_ns", "completed_round_serial", "final_native_weight_batch_sha256"]}}
    output.mkdir(parents=True, exist_ok=True)
    summary["plots"] = write_plots(output, summary, trajectories, integrations, resource_rows)
    temporary = output / "summary.json.tmp"
    temporary.write_text(json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
    temporary.replace(output / "summary.json")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-directory", type=Path, required=True,
                        help="runs/RID containing mrjq311/ and downloaded amov/, uros/ directories")
    parser.add_argument("--output-directory", type=Path,
                        help="Defaults to <run-directory>/mrjq311/analysis")
    args = parser.parse_args()
    root = args.run_directory.resolve()
    if not root.is_dir():
        parser.error("Run directory does not exist")
    output = args.output_directory.resolve() if args.output_directory else root / "mrjq311/analysis"
    try:
        summary = analyze(root, output)
    except ImportError as error:
        parser.exit(2, "Plotting dependency unavailable: %s; install matplotlib for this Python environment\n" % error)
    print(json.dumps({"run_id": root.name, "status": summary["status"],
                      "mission_success": summary["mission"].get("success"),
                      "accepted_snapshots": summary["global_display"]["accepted_snapshot_files"],
                      "summary": str(output / "summary.json"),
                      "partial_reasons": summary["partial_reasons"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
