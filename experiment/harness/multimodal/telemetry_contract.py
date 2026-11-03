"""Actual simulated-body telemetry paths; never a VIO/SLAM estimate.

The edge body keeps its native EGO odometry topic. Central consumers share the
one source-authenticated ingress copy instead of opening parallel LAN streams.
"""


def actual_odometry_topic(robot, central=False):
    if robot not in (0, 1):
        raise ValueError('Only the two configured robots are supported')
    return ('/sim_%d/ground_truth/odometry' if central else
            '/drone_%d_visual_slam/odom') % robot


def actual_odometry_caller(robot, central=False):
    if robot not in (0, 1):
        raise ValueError('Only the two configured robots are supported')
    return ('/sim_%d_multimodal_sensors' if central else
            '/drone_%d_continuous_body') % robot


def actual_odometry_description(central=False):
    path = ' through the single hub sensor ingress' if central else ''
    return ('Actual simulated body odometry' + path +
            '; evaluation and mission supervision only, never a SLAM estimate or input')
