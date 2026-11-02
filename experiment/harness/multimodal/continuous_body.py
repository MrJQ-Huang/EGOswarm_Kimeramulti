#!/usr/bin/env python3
"""Continuous local simulated body for EGO PositionCommand tracking.

Commands are references, never assigned directly to body position or velocity.
Translation integrates a stable first-order response of actual acceleration;
attitude follows the actual thrust direction and a continuous yaw controller.
The internal instantaneous angular velocity is the analytic derivative of this
attitude. Published IMU packets instead contain interval-equivalent inertial
measurements for Kimera's left-endpoint zero-order-hold preintegrator. A packet
for [t_i, t_(i+1)] is delivered one body update later, stamped t_i. No orientation
or simulator pose is supplied in that IMU packet.

This is a continuous ideal-actuation model, not a PX4/Gazebo rigid-body model.
The original EGO convention of world-frame odometry linear velocity is retained.
Odometry angular velocity is a real body rate unless the explicit legacy option
is selected. The independent body IMU always has proper sensor semantics.
"""
import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
import threading
import time
import numpy as np


def limited(vector, maximum):
    vector = np.asarray(vector, dtype=float)
    length = np.linalg.norm(vector)
    return vector*(maximum/length) if length > maximum else vector.copy()


def wrap(angle): return math.atan2(math.sin(angle), math.cos(angle))


def acceleration_response(actual, desired, dt, tau, maximum_jerk):
    """Exact bounded first-order response to a held desired acceleration.

    da/dt = limit_norm((desired-a)/tau, maximum_jerk). The entire update
    is on the line segment [actual, desired], so it cannot overshoot or wind
    up. Return a_end, analytic jerk_end, integral(a), integral((dt-t)*a).
    """
    difference = desired-actual; radius = float(np.linalg.norm(difference))
    if radius < 1e-12:
        return desired.copy(), np.zeros(3), actual*dt, actual*(.5*dt*dt)
    axis = difference/radius
    linear = min(dt, max(0., (radius-maximum_jerk*tau)/maximum_jerk))
    remaining_radius = radius-maximum_jerk*linear
    integral_radius = radius*linear-.5*maximum_jerk*linear*linear
    weighted_radius = dt*radius*linear-.5*(dt*maximum_jerk+radius)*linear*linear+maximum_jerk*linear**3/3.
    exponential = dt-linear
    one_minus = -math.expm1(-exponential/tau)
    integral_radius += remaining_radius*tau*one_minus
    weighted_radius += remaining_radius*(exponential*tau-tau*tau*one_minus)
    end_radius = remaining_radius*(1.-one_minus)
    end = desired-axis*end_radius
    jerk = axis*min(end_radius/tau, maximum_jerk)
    return end, jerk, desired*dt-axis*integral_radius, desired*(.5*dt*dt)-axis*weighted_radius


def quaternion(rotation):
    """Unit xyzw quaternion, without a scipy dependency."""
    m = rotation; trace = np.trace(m)
    if trace > 0:
        s = math.sqrt(trace+1.)*2
        q = [(m[2, 1]-m[1, 2])/s, (m[0, 2]-m[2, 0])/s, (m[1, 0]-m[0, 1])/s, .25*s]
    else:
        i = int(np.argmax(np.diag(m))); j, k = (i+1) % 3, (i+2) % 3
        s = math.sqrt(1.+m[i, i]-m[j, j]-m[k, k])*2
        q = [0., 0., 0., 0.]; q[i] = .25*s
        q[j] = (m[j, i]+m[i, j])/s; q[k] = (m[k, i]+m[i, k])/s
        q[3] = (m[k, j]-m[j, k])/s
    q = np.asarray(q); return q/np.linalg.norm(q)


def rotation_vector(rotation):
    """SO(3) log with a stable small-angle limit; result is in local axes."""
    q = quaternion(rotation)
    if q[3] < 0.: q = -q
    length = float(np.linalg.norm(q[:3]))
    if length < 1e-12: return 2.*q[:3]
    return q[:3]*(2.*math.atan2(length, float(q[3]))/length)


class LeftEndpointIntervalImu:
    """Encode the body's completed physical interval, never a motion reference.

    Kimera ImuFrontend.cpp:164-172 integrates sample i using t_(i+1)-t_i.
    Delaying delivery one sample lets us provide the completed interval at its
    left timestamp without predicting the future. Gyro reproduces its SO(3)
    increment. Specific force reproduces its velocity increment under the same
    left-rotation convention. The ordinary O(dt^3) position quadrature error is
    measured separately in the replay tests.
    """
    def __init__(self, gravity=9.8):
        self.gravity = float(gravity)
        self.previous = None

    def push(self, stamp_ns, rotation, velocity):
        current = (int(stamp_ns), np.array(rotation, copy=True), np.array(velocity, copy=True))
        previous = self.previous
        if previous is not None and current[0] <= previous[0]:
            return None
        self.previous = current
        if previous is None: return None
        first_ns, first_rotation, first_velocity = previous
        dt = (current[0]-first_ns)*1e-9
        omega = rotation_vector(first_rotation.T@current[1])/dt
        force = first_rotation.T@((current[2]-first_velocity)/dt + np.array([0., 0., self.gravity]))
        return {'stamp_ns': first_ns, 'end_stamp_ns': current[0], 'dt': dt,
                'angular_velocity': omega, 'specific_force': force}


def thrust_attitude(acceleration, jerk, yaw, yaw_rate, gravity=9.8):
    """Return R_world_body and the analytic body angular velocity.

    For normalized u, d(u/|u|)/dt = (I-uu.T) u_dot/|u|. Differentiating
    the same thrust/yaw construction used by EGO's fake body makes image
    orientation, inertial acceleration and gyroscope mutually consistent.
    """
    alpha = acceleration+np.array([0., 0., gravity])
    yc = np.array([-math.sin(yaw), math.cos(yaw), 0.])
    yc_dot = np.array([-math.cos(yaw)*yaw_rate, -math.sin(yaw)*yaw_rate, 0.])

    def unit_and_derivative(value, derivative):
        length = np.linalg.norm(value)
        if length < 1e-8: raise ValueError('Singular thrust/yaw attitude')
        unit = value/length
        return unit, (derivative-unit*np.dot(unit, derivative))/length

    xb, xb_dot = unit_and_derivative(np.cross(yc, alpha), np.cross(yc_dot, alpha)+np.cross(yc, jerk))
    yb, yb_dot = unit_and_derivative(np.cross(alpha, xb), np.cross(jerk, xb)+np.cross(alpha, xb_dot))
    zb = np.cross(xb, yb); zb_dot = np.cross(xb_dot, yb)+np.cross(xb, yb_dot)
    rotation = np.column_stack((xb, yb, zb)); derivative = np.column_stack((xb_dot, yb_dot, zb_dot))
    skew = rotation.T@derivative
    omega = .5*np.array([skew[2, 1]-skew[1, 2], skew[0, 2]-skew[2, 0], skew[1, 0]-skew[0, 1]])
    return rotation, omega


@dataclass
class Parameters:
    kp: float = 16.
    kd: float = 8.
    acceleration_limit: float = 2.5
    jerk_limit: float = 30.
    acceleration_time_constant: float = .04
    yaw_gain: float = 3.
    yaw_rate_limit: float = .7
    yaw_acceleration_limit: float = 1.5
    yaw_jerk_limit: float = 10.
    yaw_rate_time_constant: float = .15
    maximum_substep: float = .002
    command_extrapolation: float = .04
    command_timeout: float = .5
    gravity: float = 9.8


@dataclass
class Command:
    stamp: float
    received: float
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    yaw: float
    yaw_rate: float


class ContinuousBody:
    def __init__(self, position, yaw=0., parameters=None):
        self.parameters = parameters or Parameters()
        self.position = np.asarray(position, dtype=float).copy(); self.velocity = np.zeros(3)
        self.acceleration = np.zeros(3); self.jerk = np.zeros(3)
        self.yaw = float(yaw); self.yaw_rate = self.yaw_acceleration = 0.
        self.command = None; self.hold_position = self.position.copy(); self.last_time = None
        self.acceleration_saturations = 0; self.maximum_tracking_error = 0.; self.steps = 0

    def set_command(self, command):
        if not all(np.isfinite(v).all() for v in (command.position, command.velocity, command.acceleration)):
            raise ValueError('Nonfinite command')
        if not all(math.isfinite(v) for v in (command.stamp, command.received, command.yaw, command.yaw_rate)):
            raise ValueError('Nonfinite command time or yaw')
        self.command = command; self.hold_position = None

    def reference(self, now):
        c, p = self.command, self.parameters
        if c is None or now-c.received > p.command_timeout:
            if self.hold_position is None: self.hold_position = self.position.copy()
            return self.hold_position, np.zeros(3), np.zeros(3), self.yaw, 0.
        dt = max(0., min(now-c.stamp, p.command_extrapolation))
        return (c.position+c.velocity*dt+.5*c.acceleration*dt*dt,
                c.velocity+c.acceleration*dt, c.acceleration, c.yaw+c.yaw_rate*dt, c.yaw_rate)

    def advance(self, now):
        """Integrate every elapsed second; never take catch-up copies of commands."""
        if self.last_time is None: self.last_time = float(now); return False
        elapsed = now-self.last_time
        if elapsed <= 0: return False
        n = max(1, int(math.ceil(elapsed/self.parameters.maximum_substep)))
        dt = elapsed/n
        for step in range(n): self._step(dt, self.last_time+(step+1)*dt)
        self.last_time = float(now)
        return True

    def _step(self, dt, now):
        p = self.parameters; target, target_v, target_a, target_yaw, target_yaw_rate = self.reference(now)
        error = target-self.position
        desired_a = target_a+p.kp*error+p.kd*(target_v-self.velocity)
        if np.linalg.norm(desired_a) > p.acceleration_limit: self.acceleration_saturations += 1
        desired_a = limited(desired_a, p.acceleration_limit)
        next_a, jerk, delta_v, delta_p = acceleration_response(
            self.acceleration, desired_a, dt, p.acceleration_time_constant, p.jerk_limit)
        self.position += self.velocity*dt+delta_p; self.velocity += delta_v
        self.acceleration, self.jerk = next_a, jerk

        desired_yaw_rate = float(np.clip(target_yaw_rate+p.yaw_gain*wrap(target_yaw-self.yaw), -p.yaw_rate_limit, p.yaw_rate_limit))
        desired_yaw_acc = float(np.clip((desired_yaw_rate-self.yaw_rate)/p.yaw_rate_time_constant,
                                        -p.yaw_acceleration_limit, p.yaw_acceleration_limit))
        next_yaw_acc = self.yaw_acceleration+float(np.clip(desired_yaw_acc-self.yaw_acceleration,
                                              -p.yaw_jerk_limit*dt, p.yaw_jerk_limit*dt))
        yaw_jerk = (next_yaw_acc-self.yaw_acceleration)/dt
        self.yaw += self.yaw_rate*dt+.5*self.yaw_acceleration*dt*dt+yaw_jerk*(dt**3/6)
        self.yaw_rate += self.yaw_acceleration*dt+.5*yaw_jerk*dt*dt
        self.yaw_acceleration = next_yaw_acc
        self.maximum_tracking_error = max(self.maximum_tracking_error, float(np.linalg.norm(error)))
        self.steps += 1

    def attitude_and_imu(self):
        rotation, omega = thrust_attitude(self.acceleration, self.jerk, self.yaw, self.yaw_rate, self.parameters.gravity)
        force = rotation.T@(self.acceleration+np.array([0., 0., self.parameters.gravity]))
        return rotation, quaternion(rotation), omega, force

    def command_telemetry(self, now):
        """Read-only local command freshness; never changes reference or physics."""
        command = self.command
        age = None if command is None else max(0., float(now-command.received))
        return {'has_command': command is not None, 'command_received_age_s': age,
                'command_header_stamp': None if command is None else float(command.stamp),
                'command_timeout_s': self.parameters.command_timeout,
                'stale_command_hold': command is None or age > self.parameters.command_timeout,
                'command_freshness_scope': 'Local continuous_body receiver clock; not remote observer arrival time'}


def self_test(output=None):
    """150-second saturation/delay/reversal tests, not just smooth short paths."""
    parameters = Parameters()
    # tau*s^3+s^2+kd*s+kp is Hurwitz iff kd>tau*kp (all coefficients positive).
    roots = np.roots([parameters.acceleration_time_constant, 1., parameters.kd, parameters.kp])
    report = {'model_revision': 'stable-first-order-v2', 'parameters': vars(parameters),
        'linear_stability': {'characteristic': 'tau*s^3+s^2+kd*s+kp',
            'routh_kd': parameters.kd, 'routh_tau_times_kp': parameters.acceleration_time_constant*parameters.kp,
            'roots_real_imag': [[float(x.real), float(x.imag)] for x in roots],
            'passed': bool(np.all(roots.real < 0))}, 'tests': []}

    # Opaque command/reference: physical speed<1, acceleration<2 and jerk<4.
    # Seven long traversals and stationary holds fit within 150 seconds.
    distance, speed, maximum_jerk = 16., .98, 3.8
    tj = math.sqrt(speed/maximum_jerk); cruise = (distance-2*speed*tj)/speed
    travel = 4*tj+cruise; cycle = travel+3.

    def normal(t):
        leg = int(max(t, 0.)//cycle); local = max(t, 0.)-leg*cycle
        position = velocity = acceleration = 0.
        for duration, jerk in ((tj, maximum_jerk), (tj, -maximum_jerk), (cruise, 0.), (tj, -maximum_jerk), (tj, maximum_jerk)):
            h = min(duration, local)
            position += velocity*h+.5*acceleration*h*h+jerk*h**3/6
            velocity += acceleration*h+.5*jerk*h*h; acceleration += jerk*h; local -= h
            if local <= 1e-12: break
        sign = 1. if leg % 2 == 0 else -1.; u = position/distance; pi = math.pi
        xyz = np.array([-8.+position if sign > 0 else 8.-position, .4*math.sin(pi*u), 1.+.05*(1-math.cos(2*pi*u))])
        vel = np.array([sign*velocity, .4*pi*math.cos(pi*u)*velocity/distance, .1*pi*math.sin(2*pi*u)*velocity/distance])
        acc = np.array([sign*acceleration,
            .4*pi*math.cos(pi*u)*acceleration/distance-.4*pi*pi*math.sin(pi*u)*velocity**2/distance**2,
            .1*pi*math.sin(2*pi*u)*acceleration/distance+.2*pi*pi*math.cos(2*pi*u)*velocity**2/distance**2])
        return xyz, vel, acc, 0. if sign > 0 else pi, 0.

    jump_times = (10., 40., 70., 100., 130.)
    def steps(t):
        count = sum(t >= instant for instant in jump_times)
        return np.array([float(count % 2), 0., 1.]), np.zeros(3), np.zeros(3), 0., 0.

    for name, jitter, reference in [('normal_150s_reversals_stops', False, normal),
                                   ('normal_150s_delay_jitter_catchup', True, normal),
                                   ('repeated_1m_steps_150s_delay_jitter', True, steps)]:
        body = ContinuousBody(reference(0.)[0], parameters=parameters); body.advance(0.)
        rng = np.random.RandomState(20261030); t = 0.; sample = 0
        times, errors, speeds = [], [], []; maximum_a = maximum_j = maximum_omega = maximum_reference_v = maximum_reference_a = 0.
        minimum_interval = 1.; maximum_interval = 0.
        while t < 150.-1e-9:
            dt = float(rng.uniform(.006, .014)) if jitter else .01
            if jitter and sample % 80 == 30: dt = .000065
            if jitter and sample % 80 == 31: dt = .019935
            if jitter and sample % 400 == 90: dt = .04
            dt = min(dt, 150.-t)
            delay = (.02+.015*math.sin(.71*t)) if jitter else 0.
            command_time = max(0., t-delay)
            xyz, vel, acc, yaw, yaw_rate = reference(command_time)
            body.set_command(Command(command_time, t, xyz, vel, acc, yaw, yaw_rate))
            t += dt; body.advance(t)
            actual_reference = reference(t)
            times.append(t); errors.append(float(np.linalg.norm(body.position-actual_reference[0])))
            speeds.append(float(np.linalg.norm(body.velocity)))
            maximum_a = max(maximum_a, float(np.linalg.norm(body.acceleration)))
            maximum_j = max(maximum_j, float(np.linalg.norm(body.jerk)))
            maximum_reference_v = max(maximum_reference_v, float(np.linalg.norm(actual_reference[1])))
            maximum_reference_a = max(maximum_reference_a, float(np.linalg.norm(actual_reference[2])))
            minimum_interval = min(minimum_interval, dt); maximum_interval = max(maximum_interval, dt)
            if sample % 10 == 0:
                rotation, _, omega, force = body.attitude_and_imu()
                maximum_omega = max(maximum_omega, float(np.linalg.norm(omega)))
                assert np.allclose(rotation@force, body.acceleration+[0., 0., 9.8], atol=1e-9)
            sample += 1
        case = {'name': name, 'seconds': t, 'samples': sample, 'minimum_interval_s': minimum_interval,
            'maximum_interval_s': maximum_interval, 'maximum_tracking_error_m': max(errors), 'final_tracking_error_m': errors[-1],
            'maximum_speed_m_s': max(speeds), 'maximum_acceleration_m_s2': maximum_a, 'maximum_actual_jerk_m_s3': maximum_j,
            'maximum_sampled_gyro_rad_s': maximum_omega, 'maximum_reference_speed_m_s': maximum_reference_v,
            'maximum_reference_acceleration_m_s2': maximum_reference_a, 'acceleration_saturation_updates': body.acceleration_saturations}
        bounded = max(speeds) < 2.5 and maximum_a <= parameters.acceleration_limit+1e-8 and maximum_j <= parameters.jerk_limit+1e-8
        if reference is normal:
            case['passed'] = bounded and max(errors) < .3
        else:
            settle = []; times = np.asarray(times); errors = np.asarray(errors)
            for instant in jump_times:
                window = (times >= instant) & (times < instant+15.)
                tt, ee = times[window], errors[window]
                entry = {'reference_jump_time_s': instant, 'peak_error_m': float(np.max(ee))}
                for threshold, label in ((.3, 'settled_below_0p3_m_after_s'), (.05, 'settled_below_0p05_m_after_s')):
                    bad = np.flatnonzero(ee > threshold)
                    index = int(bad[-1]+1) if len(bad) else 0
                    entry[label] = float(tt[index]-instant) if index < len(tt) else None
                entry['last_5s_max_error_m'] = float(np.max(ee[tt >= instant+10.]))
                settle.append(entry)
            case['step_responses'] = settle
            case['passed'] = bounded and max(errors) < 1.1 and all(x['settled_below_0p3_m_after_s'] is not None and
                x['settled_below_0p3_m_after_s'] < 2.5 and x['settled_below_0p05_m_after_s'] is not None and
                x['settled_below_0p05_m_after_s'] < 4. and x['last_5s_max_error_m'] < .05 for x in settle)
        report['tests'].append(case)
        print(json.dumps({'test_complete': name, 'passed': case['passed'], 'maximum_error_m': case['maximum_tracking_error_m'],
                          'maximum_speed_m_s': case['maximum_speed_m_s']}), flush=True)
    stationary = ContinuousBody([3., -2., 1.]); stationary.advance(0.); stationary.advance(150.)
    _, _, w, f = stationary.attitude_and_imu()
    report['initial_stationary_150s_pass'] = bool(np.allclose(stationary.position, [3., -2., 1.]) and np.allclose(stationary.velocity, 0.) and
                                                 np.allclose(w, 0.) and np.allclose(f, [0., 0., 9.8]))
    report['passed'] = report['linear_stability']['passed'] and all(case['passed'] for case in report['tests']) and report['initial_stationary_150s_pass']
    if output:
        output.parent.mkdir(parents=True, exist_ok=True); output.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report, indent=2)); return 0 if report['passed'] else 1


def run_ros(args):
    import rospy
    from geometry_msgs.msg import Vector3Stamped
    from nav_msgs.msg import Odometry
    from quadrotor_msgs.msg import PositionCommand
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
    rospy.init_node('drone_%d_continuous_body'%args.drone_id)
    parameters = Parameters(kp=args.kp, kd=args.kd, acceleration_limit=args.acceleration_limit, jerk_limit=args.jerk_limit,
                            acceleration_time_constant=args.acceleration_time_constant)
    body = ContinuousBody(args.initial_position, args.initial_yaw, parameters); lock = threading.Lock()
    body_frame = 'sim_%d/imu'%args.drone_id
    odom_pub = rospy.Publisher('/drone_%d_visual_slam/odom'%args.drone_id, Odometry, queue_size=10)
    imu_pub = rospy.Publisher('/sim_%d/body/imu'%args.drone_id, Imu, queue_size=200)
    accel_pub = rospy.Publisher('/sim_%d/body/acceleration_world'%args.drone_id, Vector3Stamped, queue_size=10)
    status_pub = rospy.Publisher('/sim_%d/body/status'%args.drone_id, String, queue_size=1, latch=True)

    def vector(value): return np.array([value.x, value.y, value.z])
    def callback(msg):
        now = rospy.Time.now().to_sec(); stamp = msg.header.stamp.to_sec() or now
        command = Command(stamp, now, vector(msg.position), vector(msg.velocity), vector(msg.acceleration), float(msg.yaw), float(msg.yaw_dot))
        try:
            with lock: body.set_command(command)
        except ValueError as error: rospy.logerr_throttle(2., 'Ignoring invalid body reference: %s', error)

    sub = rospy.Subscriber('/drone_%d_planning/pos_cmd'%args.drone_id, PositionCommand, callback, queue_size=10, tcp_nodelay=True)
    rate = rospy.Rate(args.rate); next_status = 0.; sequence = 0
    imu_encoder = LeftEndpointIntervalImu(parameters.gravity)
    rospy.loginfo('Continuous local body: p=%s, jerk<=%.1f m/s^3; left-endpoint interval IMU with one-update delivery delay, no command pose copies', args.initial_position, parameters.jerk_limit)
    while not rospy.is_shutdown():
        stamp = rospy.Time.now(); now = stamp.to_sec()
        with lock:
            updated = body.advance(now)
            rotation, q, omega, force = body.attitude_and_imu()
            position, velocity, acceleration = body.position.copy(), body.velocity.copy(), body.acceleration.copy()
            target = body.reference(now)[0].copy()
            command_status = body.command_telemetry(now)
            interval_imu = imu_encoder.push(stamp.to_nsec(), rotation, velocity)
        if not updated: rate.sleep(); continue
        odom = Odometry(); odom.header.stamp = stamp; odom.header.frame_id = 'world'; odom.header.seq = sequence; odom.child_frame_id = body_frame
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = position.tolist()
        odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = q.tolist()
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = velocity.tolist()
        angular = acceleration if args.legacy_angular_acceleration else omega
        odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = angular.tolist()
        imu = None
        if interval_imu is not None:
            imu = Imu(); start_ns = interval_imu['stamp_ns']
            imu.header.stamp = rospy.Time(start_ns//1000000000, start_ns%1000000000)
            imu.header.frame_id = body_frame; imu.header.seq = sequence; imu.orientation_covariance[0] = -1.
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = interval_imu['angular_velocity'].tolist()
            imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = interval_imu['specific_force'].tolist()
            imu.angular_velocity_covariance = [1e-8, 0., 0., 0., 1e-8, 0., 0., 0., 1e-8]
            imu.linear_acceleration_covariance = [1e-6, 0., 0., 0., 1e-6, 0., 0., 0., 1e-6]
        actual_a = Vector3Stamped(); actual_a.header = odom.header
        actual_a.vector.x, actual_a.vector.y, actual_a.vector.z = acceleration.tolist()
        odom_pub.publish(odom)
        if imu is not None: imu_pub.publish(imu)
        accel_pub.publish(actual_a); sequence += 1
        if now >= next_status:
            status_pub.publish(String(data=json.dumps({'model': 'continuous ideal-actuation body', 'model_revision': 'stable-first-order-v2', 'drone_id': args.drone_id,
                'time': now, 'position': position.tolist(), 'velocity_world': velocity.tolist(), 'acceleration_world': acceleration.tolist(),
                'gyro_body_rad_s': omega.tolist(), 'tracking_error_m': float(np.linalg.norm(position-target)),
                'imu_sampling': 'left-endpoint interval-equivalent, one body update delayed',
                'imu_interval_s': None if interval_imu is None else interval_imu['dt'],
                'imu_angular_velocity_body_rad_s': None if interval_imu is None else interval_imu['angular_velocity'].tolist(),
                'maximum_tracking_error_m': body.maximum_tracking_error, 'acceleration_saturations': body.acceleration_saturations,
                'command_pose_is_never_copied': True, 'legacy_odom_angular_acceleration': args.legacy_angular_acceleration,
                **command_status})))
            next_status = now+1.
        rate.sleep()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--drone-id', type=int, choices=(0, 1), default=0)
    parser.add_argument('--initial-position', nargs=3, type=float, default=[0., 0., 1.])
    parser.add_argument('--initial-yaw', type=float, default=0.)
    parser.add_argument('--rate', type=float, default=100.)
    parser.add_argument('--kp', type=float, default=16.); parser.add_argument('--kd', type=float, default=8.)
    parser.add_argument('--acceleration-limit', type=float, default=2.5)
    parser.add_argument('--jerk-limit', type=float, default=30.)
    parser.add_argument('--acceleration-time-constant', type=float, default=.04)
    parser.add_argument('--legacy-angular-acceleration', action='store_true', help='Compatibility only: copy actual world acceleration into odometry angular, as the old fake body did. Body IMU remains correct.')
    parser.add_argument('--self-test', action='store_true'); parser.add_argument('--output', type=Path)
    args, unknown = parser.parse_known_args()
    if any(':=' not in item for item in unknown): parser.error('unrecognized arguments: '+' '.join(unknown))
    if args.self_test: return self_test(args.output)
    run_ros(args); return 0


if __name__ == '__main__': raise SystemExit(main())
