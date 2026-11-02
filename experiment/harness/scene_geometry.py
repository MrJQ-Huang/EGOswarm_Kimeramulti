#!/usr/bin/env python3
"""ROS-independent geometry for the two-computer warehouse scene.

points(scene, step=.12) returns deterministic surface points in scene frame.
Obstacle distances are exact signed distances for boxes and finite cylinders;
segment_hits is an explicitly sampled offline diagnostic, not a path planner.
Only Python's standard library is required.
"""

import argparse
import json
import math
from pathlib import Path


def load_scene(path=None):
    path = Path(path) if path is not None else Path(__file__).with_name("complex_scene.json")
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def _ticks(low, high, step):
    count = max(1, int(math.ceil((high - low) / step)))
    return [low + (high - low) * i / count for i in range(count + 1)]


def points(scene, step=.12):
    """Sample box/cylinder surfaces and the floor; spacing is at most step.

    Floor spacing is 0.3 m to limit transport and radius-query work. Cylinder
    caps use a Cartesian disk grid plus the side rim. Duplicate points shared
    by surfaces are removed after rounding to nine decimal places.
    """
    if not math.isfinite(step) or step <= 0:
        raise ValueError("step must be finite and positive")
    result = set()

    def add(x, y, z):
        result.add((round(float(x), 9), round(float(y), 9), round(float(z), 9)))

    bounds = scene["bounds"]
    for x in _ticks(*bounds["x"], .3):
        for y in _ticks(*bounds["y"], .3):
            add(x, y, scene.get("floor_z", 0.0))

    for obstacle in scene["obstacles"]:
        kind = obstacle["type"]
        if kind == "box":
            center, size = obstacle["center"], obstacle["size"]
            axes = [_ticks(center[i] - size[i] / 2, center[i] + size[i] / 2, step)
                    for i in range(3)]
            for fixed in range(3):
                variable = [i for i in range(3) if i != fixed]
                for edge in (axes[fixed][0], axes[fixed][-1]):
                    for first in axes[variable[0]]:
                        for second in axes[variable[1]]:
                            xyz = [0.0, 0.0, 0.0]
                            xyz[fixed] = edge
                            xyz[variable[0]], xyz[variable[1]] = first, second
                            add(*xyz)
        elif kind == "cylinder":
            x, y = obstacle["center"]
            radius = obstacle["radius"]
            bottom = obstacle.get("base_z", 0.0)
            top = bottom + obstacle["height"]
            count = max(12, int(math.ceil(2 * math.pi * radius / step)))
            for i in range(count):
                angle = 2 * math.pi * i / count
                px, py = x + radius * math.cos(angle), y + radius * math.sin(angle)
                for z in _ticks(bottom, top, step):
                    add(px, py, z)
            for dx in _ticks(-radius, radius, step):
                for dy in _ticks(-radius, radius, step):
                    if dx * dx + dy * dy <= radius * radius + 1e-12:
                        add(x + dx, y + dy, bottom)
                        add(x + dx, y + dy, top)
        else:
            raise ValueError("Unknown obstacle type: " + str(kind))
    return sorted(result)


def obstacle_clearance(point, obstacle, body_radius=0.0):
    """Exact signed distance from a spherical body to an obstacle surface.

    Positive means separated, zero means touching, negative means overlapping.
    body_radius=0 measures the clearance of the supplied point itself.
    """
    if obstacle["type"] == "box":
        q = [abs(point[i] - obstacle["center"][i]) - obstacle["size"][i] / 2
             for i in range(3)]
        outside = math.sqrt(sum(max(value, 0.0) ** 2 for value in q))
        return outside + min(max(q), 0.0) - body_radius
    if obstacle["type"] == "cylinder":
        x, y = obstacle["center"]
        radial = math.hypot(point[0] - x, point[1] - y) - obstacle["radius"]
        z_center = obstacle.get("base_z", 0.0) + obstacle["height"] / 2
        vertical = abs(point[2] - z_center) - obstacle["height"] / 2
        return (math.hypot(max(radial, 0.0), max(vertical, 0.0)) +
                min(max(radial, vertical), 0.0) - body_radius)
    raise ValueError("Unknown obstacle type: " + str(obstacle["type"]))


def clearance(scene, point, body_radius=0.0, include_floor=True):
    """Minimum signed surface clearance and corresponding obstacle id."""
    candidates = [(obstacle_clearance(point, obstacle, body_radius), obstacle["id"])
                  for obstacle in scene["obstacles"]]
    if include_floor:
        candidates.append((point[2] - scene.get("floor_z", 0.0) - body_radius, "floor"))
    if not candidates:
        return math.inf, None
    return min(candidates)


def segment_hits(scene, start, goal, samples=1001, body_radius=0.0):
    """Return obstacles touched by sampled positions on a straight segment."""
    if samples < 2:
        raise ValueError("samples must be at least 2")
    hits = set()
    for i in range(samples):
        fraction = i / (samples - 1)
        xyz = [start[j] + fraction * (goal[j] - start[j]) for j in range(3)]
        for obstacle in scene["obstacles"]:
            if obstacle_clearance(xyz, obstacle, body_radius) <= 0:
                hits.add(obstacle["id"])
    return sorted(hits)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path)
    parser.add_argument("--step", type=float, default=.12)
    args = parser.parse_args()
    scene = load_scene(args.scene)
    cloud = points(scene, args.step)
    report = {"point_count": len(cloud), "obstacle_count": len(scene["obstacles"]),
              "bounds": scene["bounds"], "scenarios": {}}
    valid = True
    for name, scenario in scene["scenarios"].items():
        endpoints = []
        for role in ("starts", "goals"):
            for drone_id, xyz in enumerate(scenario[role]):
                margin, closest = clearance(scene, xyz, include_floor=False)
                valid = valid and margin >= 1.0
                endpoints.append({"role": role, "drone_id": drone_id, "point": xyz,
                                  "obstacle_clearance_m": margin, "closest_obstacle": closest,
                                  "sphere_radius_0p2_clearance_m": margin - .2})
        straight_hits = [segment_hits(scene, start, goal)
                         for start, goal in zip(scenario["starts"], scenario["goals"])]
        valid = valid and all(straight_hits)
        report["scenarios"][name] = {"endpoints": endpoints, "straight_line_hits": straight_hits}
    report["endpoint_clearance_and_required_detour_checks_passed"] = valid
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if valid else 1


if __name__ == "__main__":
    raise SystemExit(main())
