import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
from safety_core import SafetyCore

class TrajectoryTests(unittest.TestCase):
    def setUp(self):
        self.now=1.0
        self.c=SafetyCore(lambda:self.now)
        self.c.observe();self.c.heartbeat();self.c.enable([0]*7,[1]*6)
    def healthy_tick(self,t):
        self.now=t;self.c.observe();self.c.heartbeat();self.c.tick(.004)
    def test_all_seven_joints_interpolate_and_stop_freezes(self):
        self.c.submit_trajectory([{'t_s':0,'q':[0]*7},{'t_s':2,'q':[.1]*7}],self.c.generation)
        self.healthy_tick(2.0)
        self.assertEqual(self.c.q,[.05]*7)
        self.c.stop();self.healthy_tick(2.5)
        self.assertEqual(self.c.q,[.05]*7)
        self.assertIsNone(self.c.trajectory)
    def test_speed_jump_invalid_shape_and_stale_generation_rejected(self):
        for path in [ [{'t_s':0,'q':[0]*7},{'t_s':.1,'q':[.1]*7}],
                      [{'t_s':0,'q':[.1]*7},{'t_s':2,'q':[.1]*7}],
                      [{'t_s':0,'q':[0]*14},{'t_s':2,'q':[.1]*14}],
                      [{'t_s':0,'q':[0]*7},{'t_s':2,'q':[float('nan')]*7}] ]:
            with self.assertRaises(ValueError):self.c.submit_trajectory(path,self.c.generation)
        with self.assertRaises(ValueError):
            self.c.submit_trajectory([{'t_s':0,'q':[0]*7},{'t_s':2,'q':[.1]*7}],self.c.generation-1)
    def test_completion_does_not_release_grip(self):
        self.c.submit_trajectory([{'t_s':0,'q':[0]*7},{'t_s':2,'q':[.1]*7}],self.c.generation)
        self.healthy_tick(3.0)
        self.assertTrue(self.c.motion_done);self.assertEqual(self.c.hand_q,[1]*6)
    def test_communication_loss_cancels_whole_trajectory(self):
        self.c.submit_trajectory([{'t_s':0,'q':[0]*7},{'t_s':2,'q':[.1]*7}],self.c.generation)
        self.now+=.41;self.c.observe();self.c.tick(.004)
        self.assertTrue(self.c.stopped);self.assertIsNone(self.c.trajectory)

if __name__=='__main__':unittest.main()
