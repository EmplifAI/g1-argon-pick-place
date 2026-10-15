import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from arm_sdk_command import build_command
class UpperSdkTests(unittest.TestCase):
    def test_leg_and_hand_fields_remain_inactive(self):
        q=[.1]*29;msg=build_command(q,.7)
        for i in range(12):
            m=msg.motor_cmd[i];self.assertEqual((m.mode,m.q,m.kp,m.kd,m.tau),(0,0,0,0,0))
        for i in range(30,35):
            m=msg.motor_cmd[i];self.assertEqual((m.mode,m.q,m.kp),(0,0,0))
        self.assertAlmostEqual(msg.motor_cmd[18].q,.1)
        self.assertAlmostEqual(msg.motor_cmd[29].q,.7)
