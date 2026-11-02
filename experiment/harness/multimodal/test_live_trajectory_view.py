#!/usr/bin/env python3
import math
import unittest
import numpy as np
from live_trajectory_view import TrajectoryState,SIM_T_BODY_CAMERA,initial_transform,dashed_segments


class ViewTests(unittest.TestCase):
    def make(self,maximum=30000):return TrajectoryState('new_epoch',100.,[[10.,0.,1.],[12.5,0.,1.]],max_points=maximum)
    def equal(self,a,b):np.testing.assert_allclose(a,b,atol=1e-10,rtol=0.)

    def test_actual_caller_epoch_and_world_frame(self):
        state=self.make();pose=initial_transform([10.,0.,1.],0)
        self.assertFalse(state.ingest_odometry(0,'actual',99_000_000_000,pose,'/drone_0_continuous_body','world'))
        self.assertFalse(state.ingest_odometry(0,'actual',101_000_000_000,pose,'/wrong','world'))
        self.assertFalse(state.ingest_odometry(0,'actual',101_000_000_000,pose,'/drone_0_continuous_body','map'))
        self.assertTrue(state.ingest_odometry(0,'actual',101_000_000_000,pose,'/drone_0_continuous_body','world'))
        self.assertEqual(len(state.paths['actual',0]),1)

    def test_raw_alignment_is_once_per_robot(self):
        state=self.make();first=initial_transform([3.,4.,5.],math.pi/2)
        state.ingest_odometry(0,'vio',101_000_000_000,first,'/sim_0/kimera_vio_ros/kimera_vio_ros_node','vio_odom')
        gauge=state.raw_gauges[0].copy();relative=initial_transform([1.,0.,0.],0)
        state.ingest_odometry(0,'vio',102_000_000_000,first@relative,'/sim_0/kimera_vio_ros/kimera_vio_ros_node','vio_odom')
        self.equal(state.raw_gauges[0],gauge);self.equal(state.paths['vio',0][0][1],state.initial[0]);self.equal(state.paths['vio',0][-1][1][:3,3],[11.,0.,1.])

    def test_camera_conversion_and_one_shared_optimized_gauge(self):
        state=self.make();camera=SIM_T_BODY_CAMERA
        body1=initial_transform([2.,0.,0.],0)
        state.ingest_optimized(1,[(0,101_000_000_000,body1@camera)],'/sim_1/distributed_loop_closure/distributed_loop_closure_node')
        self.assertEqual(state.paths['optimized',1],[])
        state.ingest_optimized(0,[(0,101_000_000_000,camera)],'/sim_0/distributed_loop_closure/distributed_loop_closure_node')
        self.equal(state.paths['optimized',1][0][1][:3,3],[12.,0.,1.])
        # Known sim1 start is 12.5: the 0.5 m discrepancy must remain visible.
        self.assertAlmostEqual(np.linalg.norm(state.paths['optimized',1][0][1][:3,3]-state.initial[1][:3,3]),.5)
        self.equal(state.paths['optimized',0][0][1],state.initial[0])

    def test_optimized_replaces_path_without_reanchoring(self):
        state=self.make();camera=SIM_T_BODY_CAMERA
        state.ingest_optimized(0,[(0,101_000_000_000,camera),(1,102_000_000_000,initial_transform([1,0,0],0)@camera)],'/sim_0/distributed_loop_closure/distributed_loop_closure_node');gauge=state.optimized_gauge.copy()
        state.ingest_optimized(0,[(0,101_000_000_000,initial_transform([.2,0,0],0)@camera),(1,102_000_000_000,initial_transform([1.4,0,0],0)@camera)],'/sim_0/distributed_loop_closure/distributed_loop_closure_node')
        self.assertEqual(len(state.paths['optimized',0]),2);self.equal(state.optimized_gauge,gauge);self.equal(state.paths['optimized',0][-1][1][:3,3],[11.4,0,1.])

    def test_bounded_memory_keeps_entire_time_extent(self):
        state=self.make(100)
        for i in range(350):state.ingest_odometry(0,'actual',101_000_000_000+i*100_000_000,initial_transform([10+i*.1,0,1],0),'/drone_0_continuous_body','world')
        rows=state.paths['actual',0];self.assertLessEqual(len(rows),100);self.assertEqual(rows[0][0],101_000_000_000);self.assertEqual(rows[-1][0],135_900_000_000);self.assertGreater(state.decimations['actual_0'],0)

    def test_dashes_have_gaps_and_correct_metric_length(self):
        points=dashed_segments([np.array([0.,0.,0.]),np.array([1.,0.,0.])],on=.2,off=.1)
        self.assertEqual(len(points)%2,0);length=sum(np.linalg.norm(points[i+1]-points[i]) for i in range(0,len(points),2));self.assertAlmostEqual(length,.7)


if __name__=='__main__':unittest.main(verbosity=2)
