#!/usr/bin/env python3
"""NumPy-only nearest-surface ray casting for the warehouse JSON geometry.

There is no point-cloud visibility shortcut: opaque box, finite-cylinder and
bounded-floor intersections compete for the nearest positive ray parameter.
"""
import ctypes
import os
from pathlib import Path
import numpy as np

R_BC = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


def rotation_xyzw(q):
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def body_rate(previous_q, current_q, dt):
    """Quaternion log increment / dt; never treats fake odom angular as gyro."""
    p = np.asarray(previous_q, dtype=float); p /= np.linalg.norm(p)
    q = np.asarray(current_q, dtype=float); q /= np.linalg.norm(q)
    # conjugate(previous) * current, with the quaternion sign ambiguity removed.
    v = p[3]*q[:3] - q[3]*p[:3] - np.cross(p[:3], q[:3])
    w = p[3]*q[3] + np.dot(p[:3], q[:3])
    if w < 0: v, w = -v, -w
    length = np.linalg.norm(v)
    return (2*np.arctan2(length, max(w, 0.))/length/dt)*v if length > 1e-12 else 2*v/dt


class RayScene:
    def __init__(self, description, use_native=True):
        self.scene = description
        self.obstacles = description['obstacles']
        self.kernel = None; self.backend = 'numpy'
        primitive_rows = [[0., description.get('floor_z', 0.), *description['bounds']['x'], *description['bounds']['y'], 0., 0.]]
        for obstacle in self.obstacles:
            if obstacle['type'] == 'box':
                primitive_rows.append([1., *obstacle['center'], *(np.asarray(obstacle['size'])/2.), 0.])
            elif obstacle['type'] == 'cylinder':
                primitive_rows.append([2., *obstacle['center'], obstacle.get('base_z', 0.), obstacle['radius'], obstacle['height'], 0., 0.])
        self.primitives = np.ascontiguousarray(primitive_rows, dtype=np.float32)
        library = Path(os.environ.get('RAYCAST_NATIVE_LIBRARY', str(Path(__file__).with_name('raycast_kernel.so'))))
        if use_native and library.exists() and os.environ.get('RAYCAST_FORCE_NUMPY') != '1':
            self.library = ctypes.CDLL(str(library))
            self.kernel = self.library.raycast_scene
            vector = np.ctypeslib.ndpointer(dtype=np.float32, flags='C_CONTIGUOUS')
            integer = np.ctypeslib.ndpointer(dtype=np.int32, flags='C_CONTIGUOUS')
            self.kernel.argtypes = [vector, vector, ctypes.c_int, vector, ctypes.c_int,
                                    ctypes.c_float, ctypes.c_float, vector, integer]
            self.kernel.restype = None
            self.backend = 'native-cpp'

    def cast(self, origin, directions, minimum=.05, maximum=12.):
        """Return distances, hit positions, outward normals and object indices.

        directions must be unit vectors; invalid rays have inf distance and id=-1.
        Index 0 is the floor, index k+1 is scene['obstacles'][k].
        """
        o = np.asarray(origin, dtype=np.float32)
        d = np.asarray(directions, dtype=np.float32)
        all_directions = d
        nearest = np.full(len(d), np.inf, dtype=np.float32); ids = np.full(len(d), -1, dtype=np.int32)
        if self.kernel:
            self.kernel(np.ascontiguousarray(o), np.ascontiguousarray(d), len(d), self.primitives,
                        len(self.primitives), minimum, maximum, nearest, ids)
            return self._finish(o, all_directions, nearest, ids)

        def accept(candidate, identifier, ray_indices=None):
            previous = nearest if ray_indices is None else nearest[ray_indices]
            selected = (candidate > 1e-8) & (candidate < previous)
            global_indices = np.flatnonzero(selected) if ray_indices is None else ray_indices[selected]
            nearest[global_indices] = candidate[selected]; ids[global_indices] = identifier

        with np.errstate(divide='ignore', invalid='ignore'):
            floor_t = (self.scene.get('floor_z', 0.) - o[2]) / d[:, 2]
            floor_points = o + d*floor_t[:, None]
            bounds = self.scene['bounds']
            valid = ((floor_points[:, 0] >= bounds['x'][0]) &
                     (floor_points[:, 0] <= bounds['x'][1]) &
                     (floor_points[:, 1] >= bounds['y'][0]) &
                     (floor_points[:, 1] <= bounds['y'][1]))
            accept(np.where(valid, floor_t, np.inf), 0)
            for index, obstacle in enumerate(self.obstacles, 1):
                # Conservative bounding-sphere culling is especially valuable for
                # narrow cylinders: most image pixels cannot possibly hit them.
                if obstacle['type'] == 'box':
                    sphere_center = np.asarray(obstacle['center'])
                    sphere_radius = np.linalg.norm(np.asarray(obstacle['size'])/2.)
                else:
                    sphere_center = np.array([*obstacle['center'], obstacle.get('base_z', 0.)+obstacle['height']/2.])
                    sphere_radius = np.hypot(obstacle['radius'], obstacle['height']/2.)
                offset = sphere_center-o; squared = np.dot(offset, offset)
                if np.sqrt(squared)-sphere_radius > maximum: continue
                projection = np.einsum('ij,j->i', all_directions, offset)
                ray_indices = np.flatnonzero((projection > 0) & (projection*projection >= squared-sphere_radius*sphere_radius)) if squared > sphere_radius*sphere_radius else np.arange(len(all_directions))
                if not len(ray_indices): continue
                d = all_directions[ray_indices]
                if obstacle['type'] == 'box':
                    center = np.asarray(obstacle['center'], dtype=np.float32); half = np.asarray(obstacle['size'], dtype=np.float32)/2
                    low, high = center-half, center+half
                    near = np.full(len(d), -np.inf, dtype=np.float32)
                    far = np.full(len(d), np.inf, dtype=np.float32)
                    outside_parallel = np.zeros(len(d), dtype=bool)
                    for axis in range(3):
                        parallel = np.abs(d[:, axis]) < 1e-12
                        safe_d = np.where(parallel, 1., d[:, axis])
                        first = (low[axis]-o[axis])/safe_d; second = (high[axis]-o[axis])/safe_d
                        near = np.maximum(near, np.where(parallel, -np.inf, np.minimum(first, second)))
                        far = np.minimum(far, np.where(parallel, np.inf, np.maximum(first, second)))
                        if o[axis] < low[axis] or o[axis] > high[axis]: outside_parallel |= parallel
                    candidate = np.where(near > 1e-8, near, far)
                    valid = (~outside_parallel) & (far >= near) & (far > 1e-8)
                    accept(np.where(valid, candidate, np.inf), index, ray_indices)
                elif obstacle['type'] == 'cylinder':
                    center = np.asarray(obstacle['center']); radius = obstacle['radius']
                    bottom = obstacle.get('base_z', 0.); top = bottom+obstacle['height']
                    xy = o[:2]-center; aa = np.sum(d[:, :2]**2, axis=1)
                    bb = 2*np.sum(d[:, :2]*xy, axis=1); cc = np.dot(xy, xy)-radius*radius
                    disc = bb*bb-4*aa*cc; root = np.sqrt(np.maximum(disc, 0.))
                    for sign in (-1, 1):
                        t = (-bb+sign*root)/(2*aa)
                        height = o[2]+t*d[:, 2]
                        valid = (aa > 1e-12) & (disc >= 0) & (height >= bottom) & (height <= top)
                        accept(np.where(valid, t, np.inf), index, ray_indices)
                    for height in (bottom, top):
                        t = (height-o[2])/d[:, 2]
                        projected = o[:2]+d[:, :2]*t[:, None]-center
                        valid = np.sum(projected*projected, axis=1) <= radius*radius
                        accept(np.where(valid, t, np.inf), index, ray_indices)
                else:
                    raise ValueError('Unsupported primitive: '+obstacle['type'])

        invalid = (nearest < minimum) | (nearest > maximum)
        nearest[invalid] = np.inf; ids[invalid] = -1
        return self._finish(o, all_directions, nearest, ids)

    def _finish(self, o, all_directions, nearest, ids):
        hits = np.isfinite(nearest)
        positions = o+all_directions*np.where(hits, nearest, 0.)[:, None]
        normals = np.zeros_like(all_directions); normals[ids == 0, 2] = 1.
        for index, obstacle in enumerate(self.obstacles, 1):
            selected = np.flatnonzero(ids == index)
            if not len(selected): continue
            points = positions[selected]
            if obstacle['type'] == 'box':
                offset = points-np.asarray(obstacle['center'])
                normalized = offset/(np.asarray(obstacle['size'])/2)
                axes = np.argmax(np.abs(normalized), axis=1)
                normals[selected, axes] = np.sign(offset[np.arange(len(points)), axes])
            else:
                bottom = obstacle.get('base_z', 0.); top = bottom+obstacle['height']
                cap_bottom = np.abs(points[:, 2]-bottom) < 1e-6
                cap_top = np.abs(points[:, 2]-top) < 1e-6
                normals[selected, :2] = (points[:, :2]-obstacle['center'])/obstacle['radius']
                normals[selected[cap_bottom | cap_top]] = 0.
                normals[selected[cap_bottom], 2] = -1.; normals[selected[cap_top], 2] = 1.
        return nearest, positions, normals, ids

    def colors(self, positions, normals, ids):
        """Physical, world-anchored multiscale surface tiles, stable across cameras."""
        axes = np.argmax(np.abs(normals), axis=1)
        uv = np.empty((len(ids), 2))
        for axis, pair in ((0, (1, 2)), (1, (0, 2)), (2, (0, 1))):
            selected = axes == axis; uv[selected] = positions[selected][:, pair]
        for index, obstacle in enumerate(self.obstacles, 1):
            if obstacle['type'] != 'cylinder': continue
            selected = (ids == index) & (np.abs(normals[:, 2]) < .5)
            xy = positions[selected, :2]-obstacle['center']
            uv[selected, 0] = np.arctan2(xy[:, 1], xy[:, 0])*obstacle['radius']
            uv[selected, 1] = positions[selected, 2]

        def hashed(scale):
            cells = np.floor(uv/scale).astype(np.int64)
            h = ((cells[:, 0]*73856093) ^ (cells[:, 1]*19349663) ^ ((ids+1)*83492791))
            h ^= h >> 13; h *= np.int64(1274126177); h ^= h >> 16
            return h

        fine, coarse = hashed(.12), hashed(.72)
        luminance = 38.+128.*((fine & 255)/255.)+45.*((coarse & 255)/255.)
        # A few dark tile boundaries provide sharp, repeatable physical corners.
        fraction = np.mod(uv/.36, 1.)
        luminance[np.any((fraction < .025) | (fraction > .975), axis=1)] *= .35
        rgb = np.stack([luminance*(.80+.25*((coarse >> shift) & 255)/255.) for shift in (0, 8, 16)], axis=1)
        light = np.array([-.35, -.25, .90]); light /= np.linalg.norm(light)
        illumination = .72+.28*np.maximum(0., normals@light)
        rgb *= illumination[:, None]
        rgb[ids < 0] = (32, 40, 58)
        return np.clip(rgb, 0, 255).astype(np.uint8)


def camera_rays(width, height, fx, fy, cx, cy):
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    rays = np.stack(((u-cx)/fx, (v-cy)/fy, np.ones_like(u)), axis=-1).reshape((-1, 3))
    lengths = np.linalg.norm(rays, axis=1)
    return rays/lengths[:, None], 1./lengths


def lidar_rays(channels=16, angles=180, bottom_degrees=-25., top_degrees=15.):
    elevation, azimuth = np.meshgrid(np.deg2rad(np.linspace(bottom_degrees, top_degrees, channels)),
                                    np.linspace(-np.pi, np.pi, angles, endpoint=False), indexing='ij')
    return np.stack((np.cos(elevation)*np.cos(azimuth), np.cos(elevation)*np.sin(azimuth),
                     np.sin(elevation)), axis=-1).reshape((-1, 3))
