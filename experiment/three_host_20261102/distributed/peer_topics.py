"""Fixed peer-only ROS contract, derived from the upstream RTM topic list.

This is a TCP transport replacement, not the unreleased upstream RTM. Raw
images, IMU, odometry and request_pose_graph stay on their local ROS masters.
The original ROS serialization and custom PublicPoses matrices are forwarded.
"""
from dataclasses import dataclass


LANES = ('control', 'feature', 'ego')
UPSTREAM_TOPICS = ('https://github.com/MIT-SPARK/Kimera-Distributed/blob/'
                   '542c801346e54adc202f5318c6db22a48e9e5966/'
                   'params/rtm_input_topics_robot.yaml')


@dataclass(frozen=True)
class TopicSpec:
    topic: str
    type: str
    lane: str
    callers: tuple
    latch: bool = False


def proxy_caller(robot):
    if robot not in (0, 1):
        raise ValueError('Unknown robot')
    return '/sim_%d_peer_bridge' % robot


def topic_specs(robot):
    if robot not in (0, 1):
        raise ValueError('Unknown robot')
    base = '/sim_%d' % robot
    lcd = base + '/distributed_loop_closure/distributed_loop_closure_node'
    vio = base + '/kimera_vio_ros/kimera_vio_ros_node'
    pgo = base + '/dpgo_ros_node/agent'
    specs = [TopicSpec(base + '/kimera_vio_ros/bow_query',
                       'pose_graph_tools_msgs/BowQueries', 'feature', (vio, lcd))]
    for suffix, kind, lane in (
            ('bow_requests', 'BowRequests', 'feature'),
            ('vlc_requests', 'VLCRequests', 'feature'),
            ('vlc_responses', 'VLCFrames', 'feature'),
            ('loop_closures', 'LoopClosures', 'control'),
            ('loop_ack', 'LoopClosuresAck', 'control')):
        specs.append(TopicSpec(base + '/kimera_distributed/' + suffix,
                               'pose_graph_tools_msgs/' + kind, lane, (lcd,)))
    # Only robot zero publishes the native global-frame corrector.
    if robot == 0:
        specs.append(TopicSpec(base + '/kimera_distributed/pose_corrector',
                               'geometry_msgs/Pose', 'control', (lcd,), True))
    for suffix, kind in (
            ('lifting_matrix', 'dpgo_ros/MatrixMsg'),
            ('status', 'dpgo_ros/Status'),
            ('command', 'dpgo_ros/Command'),
            ('anchor', 'dpgo_ros/PublicPoses'),
            ('public_poses', 'dpgo_ros/PublicPoses'),
            ('public_measurements', 'dpgo_ros/RelativeMeasurementList'),
            ('measurement_weights', 'dpgo_ros/RelativeMeasurementWeights'),
            ('path', 'nav_msgs/Path'),
            ('loop_closures', 'visualization_msgs/Marker')):
        specs.append(TopicSpec(base + '/dpgo_ros_node/' + suffix, kind,
                               'control', (pgo,)))
    specs.extend([
        TopicSpec(base + '/kimera_distributed/optimized_path', 'nav_msgs/Path',
                  'control', (lcd,)),
        TopicSpec('/broadcast_bspline', 'traj_utils/Bspline', 'ego',
                  ('/drone_%d_ego_planner_node' % robot,)),
        TopicSpec(base + '/body/status', 'std_msgs/String', 'ego',
                  ('/drone_%d_continuous_body' % robot,)),
        TopicSpec(base + '/mission/status', 'std_msgs/String', 'ego',
                  ('/sim_%d_local_mission' % robot,)),
        # The bridge owns the local heartbeat; its session, not a received
        # arbitrary Header, controls connected_peer_ids.
        TopicSpec(base + '/heartbeat', 'std_msgs/Header', 'ego',
                  (proxy_caller(robot),)),
    ])
    return {s.topic: s for s in specs}


def requires_native_subscriber(spec, source_robot):
    """Core native peer inputs must not be ACKed into an empty publisher.

    Upstream RTM also lists visualization outputs and both directions of
    asymmetric LCD topics. Those unused mirrors are allowed without consumers;
    ordinary ROS publication is their complete contract. Mission/telemetry uses
    fresh repeated state rather than retained control packets.
    """
    topic = spec.topic
    if spec.lane == 'ego' or topic.endswith(('/path', '/optimized_path')):
        return False
    if '/dpgo_ros_node/' in topic:
        return not topic.endswith('/loop_closures')
    if topic.endswith('/pose_corrector'):
        return True
    if topic.endswith(('/bow_requests', '/vlc_requests', '/loop_closures')):
        return source_robot == 0
    if topic.endswith(('/bow_query', '/vlc_responses', '/loop_ack')):
        return source_robot == 1
    return False


def validate_source(spec, caller):
    if caller not in spec.callers:
        raise ValueError('Rejected source caller for ' + spec.topic)
    # The shared broadcast topic must never re-export a received proxy sample.
    if spec.topic == '/broadcast_bspline' and caller.endswith('_peer_bridge'):
        raise ValueError('Refusing a bridge forwarding loop')
