import json
import sys
import tempfile
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from diagnostics import snapshot

class DiagnosticsTests(unittest.TestCase):
    def test_log_tail_skips_partial_and_observation_lines(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            (root/'logs').mkdir()
            with (root/'logs/argon-actions.jsonl').open('w') as f:
                f.write('broken\n')
                for i in range(60):
                    f.write(json.dumps({'event':'model_response','pc_time_s':i,'action':{'action':'observe'}})+'\n')
                f.write(json.dumps({'event':'observation','state':{'large':'data'}})+'\n')
                f.write('{partial')
            data=snapshot(root)
            self.assertEqual(len(data['events']),40)
            self.assertEqual(data['events'][0]['pc_time_s'],20)
            self.assertEqual(data['events'][-1]['pc_time_s'],59)
            self.assertIsNone(data['scene'])
            self.assertIsNone(data['depth_age_s'])
