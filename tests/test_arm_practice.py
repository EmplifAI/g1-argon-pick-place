import json
import sys
import threading
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from coarse_primitives import PickPlaceTools
from left_arm_kinematics import LeftArmKinematics

class PracticeTests(unittest.TestCase):
    def setUp(self):
        root=Path(__file__).resolve().parents[1]
        self.tools=object.__new__(PickPlaceTools)
        self.tools.kin=LeftArmKinematics(root)
        self.tools.trial_origin=None;self.tools.trial_count=0
        self.state={'motors':[{'q':0} for _ in range(29)],'left_hand':{'q':[1,.913,.968,1,1,.805]}}
        for i,v in enumerate([-.15,.36,.55,.96,.36,-.75,-.1]):self.state['motors'][15+i]['q']=v
        self.state['target_q']=[m['q'] for m in self.state['motors'][15:22]]
    def test_candidates_raise_arm_landmarks_and_limit_displacement(self):
        candidates=self.tools.arm_trial_candidates(self.state)
        self.assertTrue(candidates)
        for c in candidates:
            d=np.array(c['predicted_landmark_delta_m'])
            self.assertGreaterEqual(d[:,2].min(),-1e-5)
            self.assertLessEqual(np.linalg.norm(d,axis=1).max(),.025)
            self.assertIn(c['joint_id'],range(15,22))
    def test_trial_budget_stops_before_network_or_motion(self):
        self.tools.trial_count=3
        self.tools.robot=lambda *a:self.fail('Network/motion after trial budget')
        with self.assertRaises(ValueError):self.tools.trial_left_arm(16,1)
    def test_hand_enabled_blocks_arm_only_trial(self):
        self.tools.check_running=lambda _:dict(self.state,hand_write_enabled=True)
        self.tools.robot=lambda *a:self.fail('Hand-enabled trial sent motion')
        with self.assertRaises(ValueError):self.tools.trial_left_arm(16,1)
