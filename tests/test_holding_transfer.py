import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as S
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from holding_transfer import validate_checkpoint

class HoldingTransferTests(unittest.TestCase):
    def setUp(self):
        self.c={'boot_id':'same','mechanically_supported_confirmed':True,'saved_robot_monotonic_s':100,
            'stopped':True,'controller_active':True,'hand_write_enabled':False,'disabled_left_hand_channels':[0,1],
            'target_q':[.01]*7,'commanded_hand_q':[.8]*6,'generation':12}
        self.s=S(mode_machine=5,motor_state=[S(mode=1 if i in range(15,22) else 0,motorstate=0,q=0,dq=0,temperature=[40,40]) for i in range(29)])
    def test_transfer_keeps_exact_hold_and_invalidates_old_generation(self):
        q,h,g=validate_checkpoint(self.c,self.s,'same',101)
        self.assertEqual(q,self.c['target_q']);self.assertEqual(h,self.c['commanded_hand_q']);self.assertEqual(g,13)
    def test_boot_expiry_other_motor_and_support_guard(self):
        for patch in [{'boot_id':'old'},{'saved_robot_monotonic_s':80},{'mechanically_supported_confirmed':False},{'hand_write_enabled':True}]:
            with self.assertRaises(ValueError):validate_checkpoint(dict(self.c,**patch),self.s,'same',101)
        self.s.motor_state[22].mode=1
        with self.assertRaises(ValueError):validate_checkpoint(self.c,self.s,'same',101)
