#!/usr/bin/env python3
"""Interpolate native VIO keyframe rotations using measured IMU rotations.

This is a bounded, two-ended time interpolator, not another pose estimator.
It never changes a native keyframe, translation, graph or map gauge. Both
native orientation endpoints constrain the integrated gyro curve. This avoids
inventing constant angular velocity between keyframes during a rapid turn.

The supported sampling contract is explicit: angular_velocity at t_i is the
left-endpoint, interval-equivalent angular rate for [t_i, t_{i+1}). This matches
the simulator's actual body IMU and Kimera's left-endpoint preintegration.
No instantaneous/midpoint assumption is substituted, and missing samples never
silently fall back to SLERP. No IMU orientation field is read.
"""
import bisect
import math

import numpy as np


class GyroCoverageError(ValueError):
    def __init__(self, message, kind):
        super().__init__(message)
        self.kind = kind


def quaternion_multiply(a, b):
    """Hamilton product, xyzw; rotations map body vectors into the parent."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw*bx+ax*bw+ay*bz-az*by,
                     aw*by-ax*bz+ay*bw+az*bx,
                     aw*bz+ax*by-ay*bx+az*bw,
                     aw*bw-ax*bx-ay*by-az*bz], dtype=float)


def quaternion_inverse(q):
    result = np.asarray(q, dtype=float).copy()
    result[:3] *= -1.
    return result


def rotation_vector_quaternion(vector):
    vector = np.asarray(vector, dtype=float)
    angle = float(np.linalg.norm(vector))
    if angle < 1e-10:
        result = np.r_[.5*vector, 1.-angle*angle/8.]
        return result/np.linalg.norm(result)
    return np.r_[vector*(math.sin(angle/2.)/angle), math.cos(angle/2.)]


def quaternion_matrix(q):
    q = np.asarray(q, dtype=float)
    q = q/np.linalg.norm(q)
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def matrix_quaternion(rotation):
    rotation = np.asarray(rotation, dtype=float)
    if (rotation.shape != (3, 3) or not np.isfinite(rotation).all() or
            not np.allclose(rotation.T@rotation, np.eye(3), atol=1e-7) or
            abs(float(np.linalg.det(rotation))-1.) > 1e-7):
        raise ValueError('Native rotation endpoint must belong to SO(3)')
    candidates = np.array([1+rotation[0, 0]-rotation[1, 1]-rotation[2, 2],
        1-rotation[0, 0]+rotation[1, 1]-rotation[2, 2],
        1-rotation[0, 0]-rotation[1, 1]+rotation[2, 2], 1+np.trace(rotation)])
    index = int(np.argmax(candidates))
    q = np.zeros(4)
    q[index] = .5*math.sqrt(max(0., candidates[index]))
    divisor = 4.*q[index]
    if index == 3:
        q[:3] = [rotation[2, 1]-rotation[1, 2], rotation[0, 2]-rotation[2, 0],
                 rotation[1, 0]-rotation[0, 1]]
        q[:3] /= divisor
    elif index == 0:
        q[1:] = [(rotation[0, 1]+rotation[1, 0])/divisor,
                 (rotation[0, 2]+rotation[2, 0])/divisor,
                 (rotation[2, 1]-rotation[1, 2])/divisor]
    elif index == 1:
        q[0] = (rotation[0, 1]+rotation[1, 0])/divisor
        q[2] = (rotation[1, 2]+rotation[2, 1])/divisor
        q[3] = (rotation[0, 2]-rotation[2, 0])/divisor
    else:
        q[0] = (rotation[0, 2]+rotation[2, 0])/divisor
        q[1] = (rotation[1, 2]+rotation[2, 1])/divisor
        q[3] = (rotation[1, 0]-rotation[0, 1])/divisor
    q /= np.linalg.norm(q)
    return q if q[3] >= 0 else -q


def fractional_quaternion(q, fraction):
    q = np.asarray(q, dtype=float)
    q = q/np.linalg.norm(q)
    if q[3] < 0:
        q = -q
    vector_norm = float(np.linalg.norm(q[:3]))
    if vector_norm < 1e-12:
        return rotation_vector_quaternion(2.*fraction*q[:3])
    half_angle = math.atan2(vector_norm, float(q[3]))
    return np.r_[q[:3]*(math.sin(fraction*half_angle)/vector_norm),
                 math.cos(fraction*half_angle)]


class GyroRotationHistory:
    """Monotonic actual gyro samples with bounded storage and explicit gaps."""
    def __init__(self, maximum_gap_ns, maximum_samples=24000,
                 sample_convention='left_endpoint_interval_equivalent'):
        if sample_convention != 'left_endpoint_interval_equivalent':
            raise ValueError('Unsupported gyro sample contract; declare left-endpoint interval-equivalent samples')
        if maximum_gap_ns <= 0 or maximum_samples < 3:
            raise ValueError('Positive gyro gap and at least three retained samples required')
        self.maximum_gap_ns = int(maximum_gap_ns)
        self.maximum_samples = int(maximum_samples)
        self.sample_convention = sample_convention
        self.stamps, self.omega, self.cumulative = [], [], []
        self.samples_seen = 0
        self.maximum_sample_gap_ns = 0
        self.maximum_used_gap_ns = 0
        self.maximum_endpoint_residual_rad = 0.
        self.queries = 0

    def put(self, stamp_ns, omega_xyz):
        stamp_ns = int(stamp_ns)
        omega = np.asarray(omega_xyz, dtype=float)
        if stamp_ns <= 0 or omega.shape != (3,) or not np.isfinite(omega).all():
            raise ValueError('Actual gyro sample requires positive timestamp and finite xyz')
        if self.stamps and stamp_ns <= self.stamps[-1]:
            if stamp_ns == self.stamps[-1] and np.array_equal(omega, self.omega[-1]):
                return
            raise ValueError('Actual gyro timestamps must be monotonic; no sample replacement')
        cumulative = np.array([0., 0., 0., 1.])
        if self.stamps:
            gap = stamp_ns-self.stamps[-1]
            self.maximum_sample_gap_ns = max(self.maximum_sample_gap_ns, gap)
            # A gap is retained and rejected only if an interpolation crosses
            # it. This permits explicit omission and recovery after a dropout.
            increment = rotation_vector_quaternion(self.omega[-1]*(gap*1e-9))
            cumulative = quaternion_multiply(self.cumulative[-1], increment)
            cumulative /= np.linalg.norm(cumulative)
        self.stamps.append(stamp_ns)
        self.omega.append(omega.copy())
        self.cumulative.append(cumulative)
        self.samples_seen += 1
        # Prune in chunks, so a 100 Hz transport callback does O(1) work.
        if len(self.stamps) > self.maximum_samples+512:
            count = len(self.stamps)-self.maximum_samples
            del self.stamps[:count]
            del self.omega[:count]
            del self.cumulative[:count]

    def _at(self, stamp_ns):
        if not self.stamps or stamp_ns > self.stamps[-1]:
            raise GyroCoverageError('Waiting for actual upper gyro bracket', 'future')
        if stamp_ns < self.stamps[0]:
            raise GyroCoverageError('Actual lower gyro bracket is unavailable', 'past')
        index = bisect.bisect_right(self.stamps, stamp_ns)-1
        if self.stamps[index] == stamp_ns:
            return self.cumulative[index].copy()
        # The right sample exists, although its rate is not used. Its stamp
        # proves the complete left-endpoint interval has actually arrived.
        increment = rotation_vector_quaternion(self.omega[index]*((stamp_ns-self.stamps[index])*1e-9))
        return quaternion_multiply(self.cumulative[index], increment)

    def _require_coverage(self, start_ns, end_ns):
        self._at(start_ns)
        self._at(end_ns)
        lo = bisect.bisect_right(self.stamps, start_ns)-1
        hi = bisect.bisect_left(self.stamps, end_ns)
        gaps = [self.stamps[k+1]-self.stamps[k] for k in range(lo, hi)]
        maximum = max(gaps, default=0)
        if maximum > self.maximum_gap_ns:
            raise GyroCoverageError('Actual gyro gap %.6fs exceeds %.6fs' %
                (maximum*1e-9, self.maximum_gap_ns*1e-9), 'gap')
        self.maximum_used_gap_ns = max(self.maximum_used_gap_ns, maximum)

    def rotation_at(self, rotation0, rotation1, start_ns, end_ns, query_ns):
        """Return rotation at query, preserving both native endpoints exactly.

        For interior queries the entire keyframe interval must have gyro
        coverage. Integrate the measured body angular rate, then distribute
        only the SO(3) endpoint residual over that measured curve. This also
        absorbs a slowly varying interval bias without inventing a bias state.
        """
        start_ns, end_ns, query_ns = int(start_ns), int(end_ns), int(query_ns)
        if start_ns >= end_ns or not start_ns <= query_ns <= end_ns:
            raise ValueError('Query must lie inside an ordered native keyframe bracket')
        q0, q1 = matrix_quaternion(rotation0), matrix_quaternion(rotation1)
        if query_ns == start_ns:
            return np.asarray(rotation0, dtype=float).copy()
        if query_ns == end_ns:
            return np.asarray(rotation1, dtype=float).copy()
        self._require_coverage(start_ns, end_ns)
        inverse_start = quaternion_inverse(self._at(start_ns))
        measured_query = quaternion_multiply(inverse_start, self._at(query_ns))
        measured_end = quaternion_multiply(inverse_start, self._at(end_ns))
        predicted_query = quaternion_multiply(q0, measured_query)
        predicted_end = quaternion_multiply(q0, measured_end)
        residual = quaternion_multiply(quaternion_inverse(predicted_end), q1)
        residual /= np.linalg.norm(residual)
        self.maximum_endpoint_residual_rad = max(self.maximum_endpoint_residual_rad,
            2.*math.atan2(float(np.linalg.norm(residual[:3])), abs(float(residual[3]))))
        fraction = (query_ns-start_ns)/(end_ns-start_ns)
        result = quaternion_multiply(predicted_query, fractional_quaternion(residual, fraction))
        self.queries += 1
        return quaternion_matrix(result)
