import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from left_arm_kinematics import LeftArmKinematics

class KinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.k=LeftArmKinematics(Path(__file__).resolve().parents[1])
        cls.s={'motors':[{'q':0.0} for _ in range(29)],'left_hand':{'q':[1,1,1,1,1,.83]}}
        for i,v in enumerate([.2,.2,0,.9,0,0,0]):cls.s['motors'][15+i]['q']=v
    def test_ik_round_trip_and_nonleft_configuration_preserved(self):
        full=self.k.configuration(self.s);pose=self.k.pose(full)
        solved=self.k.solve(full,pose.translation,pose.rotation)
        np.testing.assert_allclose(solved,full[self.k.left],atol=1e-5)
        before=full.copy();self.k.pose(full,solved);np.testing.assert_array_equal(full,before)
    def test_small_cartesian_path_has_bounded_joint_speed(self):
        full=self.k.configuration(self.s);p=self.k.pose(full)
        plan=self.k.plan(self.s,p.translation+np.array([.005,0,.002]),p.rotation)
        for a,b in zip(plan['waypoints'],plan['waypoints'][1:]):
            speed=max(abs(np.array(b['q'])-a['q']))/(b['t_s']-a['t_s'])
            self.assertLessEqual(speed,.060001)
    def test_unverified_scene_cannot_execute(self):
        with self.assertRaises(ValueError):self.k.collision_check({}, {'calibration_verified':False})
    def test_unreachable_target_rejected(self):
        full=self.k.configuration(self.s)
        with self.assertRaises(ValueError):self.k.solve(full,[10,10,10])

if __name__=='__main__':unittest.main()
