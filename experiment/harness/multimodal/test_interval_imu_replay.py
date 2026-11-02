#!/usr/bin/env python3
"""Independent inertial integration checks using real commands and body timing.

No ROS publishers or estimator corrections. The body sees command references;
the independent integrator sees only initial state plus inertial packets.
"""
import argparse
import ast
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import time
import numpy as np


def load_body(path):
    spec = importlib.util.spec_from_file_location('body_interval_under_test', str(path))
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def exp_so3(w):
    angle = float(np.linalg.norm(w))
    k = np.array([[0., -w[2], w[1]], [w[2], 0., -w[0]], [-w[1], w[0], 0.]])
    if angle < 1e-8: return np.eye(3)+k+.5*k@k
    return np.eye(3)+math.sin(angle)/angle*k+(1-math.cos(angle))/(angle*angle)*k@k


class ImuOnlyIntegrator:
    """Native left-endpoint ZOH convention, independently implemented."""
    def __init__(self, p, v, rotation):
        self.p = np.array(p, copy=True)
        self.v = np.array(v, copy=True)
        self.r = np.array(rotation, copy=True)

    def step(self, force, gyro, dt):
        world_a = self.r@force + np.array([0., 0., -9.8])
        self.p += self.v*dt + .5*world_a*dt*dt
        self.v += world_a*dt
        self.r = self.r@exp_so3(gyro*dt)


def summaries(values):
    a = np.asarray(values)
    return {'rmse': float(np.sqrt(np.mean(a*a))), 'max': float(np.max(a)), 'last': float(a[-1])}


def check_case(module, name, times_ns, commands, initial_position, initial_yaw=0., camera_ns=None):
    base_ns = int(times_ns[0])
    body = module.ContinuousBody(initial_position, initial_yaw)
    encoder = module.LeftEndpointIntervalImu()
    body.advance(0.)
    r, _, old_gyro, old_force = body.attitude_and_imu()
    encoder.push(base_ns, r, body.velocity)
    fixed = ImuOnlyIntegrator(body.position, body.velocity, r)
    original = ImuOnlyIntegrator(body.position, body.velocity, r)
    cursor = 0
    errors = {key: [] for key in ['new_rot_deg', 'old_rot_deg', 'new_velocity_m_s', 'new_position_m', 'old_position_m']}
    max_dt = 0.
    now_before = 0.
    left_stamp_ok = True
    positions = []
    state_rotations = [r.copy()]
    packets = []
    for stamp_ns in times_ns[1:]:
        now = (int(stamp_ns)-base_ns)*1e-9
        while cursor < len(commands) and commands[cursor]['received'] <= now:
            c = commands[cursor]
            body.set_command(module.Command(c['stamp'], c['received'], np.array(c['position']),
                                           np.array(c['velocity']), np.array(c['acceleration']), c['yaw'], c['yaw_rate']))
            cursor += 1
        body.advance(now)
        r, _, gyro, force = body.attitude_and_imu()
        packet = encoder.push(int(stamp_ns), r, body.velocity)
        dt = now-now_before
        max_dt = max(max_dt, dt)
        left_stamp_ok &= packet['stamp_ns'] < packet['end_stamp_ns'] == int(stamp_ns)
        fixed.step(packet['specific_force'], packet['angular_velocity'], packet['dt'])
        original.step(old_force, old_gyro, dt)
        errors['new_rot_deg'].append(math.degrees(float(np.linalg.norm(module.rotation_vector(fixed.r.T@r)))))
        errors['old_rot_deg'].append(math.degrees(float(np.linalg.norm(module.rotation_vector(original.r.T@r)))))
        errors['new_velocity_m_s'].append(float(np.linalg.norm(fixed.v-body.velocity)))
        errors['new_position_m'].append(float(np.linalg.norm(fixed.p-body.position)))
        errors['old_position_m'].append(float(np.linalg.norm(original.p-body.position)))
        positions.append(body.position.copy())
        state_rotations.append(r.copy())
        packets.append(packet)
        old_gyro, old_force = gyro.copy(), force.copy()
        now_before = now
    result = {'name': name, 'duration_s': now_before, 'body_samples': len(times_ns), 'commands_applied': cursor,
              'max_body_interval_s': max_dt, 'left_stamp_delivery_verified': bool(left_stamp_ok),
              'metrics': {key: summaries(value) for key, value in errors.items()},
              'body_net_displacement_m': float(np.linalg.norm(body.position-initial_position)),
              'body_path_length_m': float(np.sum(np.linalg.norm(np.diff(np.asarray(positions), axis=0), axis=1)))}
    if camera_ns is not None:
        body_ns = set(int(v) for v in times_ns)
        result['actual_camera_boundaries'] = {'count': len(camera_ns),
                                               'exact_body_imu_stamp_matches': sum(int(v) in body_ns for v in camera_ns),
                                               'missing_body_stamps': sum(int(v) not in body_ns for v in camera_ns),
                                               'native_border_rule': 'getImuDataInterpolatedBorders; exact stored camera timestamps need no border interpolation'}
        # The data provider interpolates only the borders. Replay that rule and
        # native left integration across consecutive real camera windows. This
        # also checks the two image headers whose GT packet was absent in the
        # bag, using an SO(3) interpolation of the completed physical interval.
        import bisect
        stamps = [v['stamp_ns'] for v in packets]
        rates = [v['angular_velocity'] for v in packets]
        def boundary_rate(stamp):
            i = bisect.bisect_left(stamps, stamp)
            if i < len(stamps) and stamps[i] == stamp: return rates[i]
            if i == 0 or i == len(stamps): return None
            f = (stamp-stamps[i-1])/(stamps[i]-stamps[i-1])
            return (1.-f)*rates[i-1]+f*rates[i]
        def physical_rotation(stamp):
            i = bisect.bisect_left(times_ns, stamp)
            if i < len(times_ns) and times_ns[i] == stamp: return state_rotations[i]
            if i == 0 or i == len(times_ns): return None
            f = (stamp-int(times_ns[i-1]))/(int(times_ns[i])-int(times_ns[i-1]))
            return state_rotations[i-1]@exp_so3(module.rotation_vector(state_rotations[i-1].T@state_rotations[i])*f)
        borders = sorted(set(int(v) for v in camera_ns if stamps[0] <= int(v) <= stamps[-1]))
        result['actual_camera_boundaries']['outside_recorded_body_coverage'] = sum(int(v) < stamps[0] or int(v) > stamps[-1] for v in camera_ns)
        result['actual_camera_boundaries']['interpolated_borders_within_coverage'] = sum(v not in body_ns for v in borders)
        border_errors = []
        if len(borders) > 1:
            integrated = physical_rotation(borders[0]).copy()
            for first, last in zip(borders, borders[1:]):
                inside = [s for s in stamps if first < s < last]
                sample_stamps = [first]+inside+[last]
                for left, right in zip(sample_stamps, sample_stamps[1:]):
                    integrated = integrated@exp_so3(boundary_rate(left)*(right-left)*1e-9)
                border_errors.append(math.degrees(float(np.linalg.norm(module.rotation_vector(integrated.T@physical_rotation(last))))))
        result['actual_camera_boundaries']['native_left_integrated_rotation_error_deg'] = summaries(border_errors)
        result['actual_camera_boundaries']['evaluated_windows'] = len(border_errors)
    result['passed'] = bool(left_stamp_ok and result['metrics']['new_rot_deg']['max'] < 1e-5
                            and result['metrics']['new_velocity_m_s']['max'] < 1e-6
                            and result['metrics']['new_position_m']['max'] < .03)
    print(json.dumps({'case_completed': name, 'passed': result['passed'], 'metrics': result['metrics']}), flush=True)
    return result


def synthetic_commands(duration):
    commands = []
    for t in np.arange(0., duration+.001, .02):
        # Real PD transients, turns and abrupt replanning changes; the command
        # is never substituted for actual position/velocity in the body.
        stage = int(t//6)
        anchor = [[0,0,1], [3,0,1], [3,3,1], [0,3,1], [0,0,1], [-2,-1,1]][stage % 6]
        commands.append({'received': float(t), 'stamp': float(t)-.015,
                         'position': anchor, 'velocity': [0,0,0], 'acceleration': [0,0,0],
                         'yaw': (stage % 4)*math.pi/2, 'yaw_rate': 0.})
    return commands


def unchanged_physics(original, revised):
    names = ('acceleration_response', 'thrust_attitude', 'ContinuousBody')
    a = {v.name: ast.dump(v, include_attributes=False) for v in ast.parse(original.read_text()).body if isinstance(v, (ast.FunctionDef, ast.ClassDef)) and v.name in names}
    b = {v.name: ast.dump(v, include_attributes=False) for v in ast.parse(revised.read_text()).body if isinstance(v, (ast.FunctionDef, ast.ClassDef)) and v.name in names}
    return a == b and len(a) == len(names)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--module', type=Path, required=True)
    p.add_argument('--baseline', type=Path, required=True)
    p.add_argument('--actual-inputs', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    module = load_body(a.module)
    actual = json.loads(a.actual_inputs.read_text())
    base_ns = actual['body'][0]['stamp_ns']
    commands = [{'received': (v['received_stamp_ns']-base_ns)*1e-9,
                 'stamp': (v['header_stamp_ns']-base_ns)*1e-9,
                 **{k: v[k] for k in ['position','velocity','acceleration','yaw','yaw_rate']}}
                for v in actual['commands']]
    results = [check_case(module, 'actual_AMOV_EGO_commands_and_recorded_body_schedule',
                          [v['stamp_ns'] for v in actual['body']], commands, actual['body'][0]['position'],
                          camera_ns=actual['camera_stamps_ns'])]
    # Compare the same complete motion under regular and irregular body output
    # schedules. Max 60 ms gaps force multiple physical substeps and expose the
    # original instantaneous-jerk sampling problem.
    for jitter in (False, True):
        times = [0]
        rng = np.random.RandomState(20261102)
        while times[-1] < 48_000_000_000:
            interval = int(rng.uniform(.006, .014)*1e9) if jitter else 10_000_000
            if jitter and len(times) % 317 == 0: interval = 60_000_000
            times.append(min(48_000_000_000, times[-1]+interval))
        results.append(check_case(module, 'PD_turns_replanning_steps_'+('jitter' if jitter else '100Hz'),
                                  times, synthetic_commands(48.), np.array([0.,0.,1.])))
    report = {'wall': time.time(), 'scope': 'simulated interval IMU generation, no estimator pose input or GT correction',
              'native_sampling_source': 'ImuFrontend.cpp:164-172 and :208-213; left endpoint ZOH',
              'physics_ast_unchanged': unchanged_physics(a.baseline, a.module),
              'source_sha256': hashlib.sha256(a.module.read_bytes()).hexdigest(),
              'input_sha256': hashlib.sha256(a.actual_inputs.read_bytes()).hexdigest(),
              'cases': results}
    report['passed'] = bool(report['physics_ast_unchanged'] and all(v['passed'] for v in results))
    a.output.parent.mkdir(exist_ok=True, parents=True)
    a.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__': raise SystemExit(main())
