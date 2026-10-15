import struct
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from inspire_serial import InspireLeft

class IsolationTests(unittest.TestCase):
    def test_broken_finger_registers_are_not_written(self):
        hand=object.__new__(InspireLeft);calls=[]
        hand.request=lambda *args:(calls.append(args) or b'\x01')
        hand.set_positions([.01,.02,.8,.7,.6,.5])
        op,addr,data,size=calls[0]
        self.assertEqual((op,addr,size),(18,1490,9))
        self.assertEqual(struct.unpack('<4h',data),(800,700,600,500))
    def test_bad_isolation_fails_before_write(self):
        hand=object.__new__(InspireLeft)
        hand.request=lambda *args:self.fail('Unexpected write')
        with self.assertRaises(ValueError):hand.set_positions([1]*6,[])
