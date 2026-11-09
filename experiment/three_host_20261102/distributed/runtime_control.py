#!/usr/bin/env python3
"""Finite commands against this edge's own ROS master only."""
import argparse
import json
import time


def main():
    import rospy
    from std_msgs.msg import String
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=('reset', 'probe'))
    p.add_argument('--robot-id', type=int, choices=(0, 1), required=True)
    p.add_argument('--run-id', required=True)
    p.add_argument('--cutoff', type=float, required=True)
    a = p.parse_args(rospy.myargv()[1:])
    rospy.init_node('distributed_runtime_control', anonymous=True)
    if a.action == 'reset':
        # This is called once before either estimator is started.
        rospy.set_param('/two_pc/experiment_run_id', a.run_id)
        rospy.set_param('/two_pc/experiment_cutoff', a.cutoff)
        rospy.set_param('/use_sim_time', False)
        rospy.set_param('/rosout_disable_topics_generation', True)
        pub = rospy.Publisher('/two_pc/scenario_control', String, queue_size=1)
        deadline = time.monotonic() + 20
        while pub.get_num_connections() != 1 and time.monotonic() < deadline:
            time.sleep(.1)
        if pub.get_num_connections() != 1: raise RuntimeError('Expected one local supervisor')
        pub.publish(String(data=json.dumps({'command': 'reset', 'scenario': 'broad_loop',
                    'run_id': a.run_id + '_mission', 'continuous_body': True})))
        time.sleep(1)
        print(json.dumps({'reset_delivered': True, 'robot_id': a.robot_id, 'run_id': a.run_id}))
    else:
        code, message, state = rospy.get_master().getSystemState()
        if code != 1: raise RuntimeError(message)
        print(json.dumps({'robot_id': a.robot_id, 'run_id': a.run_id,
                          'publishers': state[0], 'subscribers': state[1], 'services': state[2]}))


if __name__ == '__main__': main()
