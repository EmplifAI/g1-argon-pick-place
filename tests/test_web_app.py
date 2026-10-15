"""Offline integration tests: real HTTP server, mocked provider/hardware only."""
import base64
import io
import json
import sys
import threading
import unittest
import urllib.error
import urllib.request
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from web_app import Console, Handler, ThreadingHTTPServer
from agent_adapter import validate_practice_action, GeminiAdapter
from gemini_client import call, output_text


class WebTests(unittest.TestCase):
    def setUp(self):
        self.server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.app = self.server.console = Console('http://127.0.0.1:1', 'http://127.0.0.1:2')
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.url = 'http://127.0.0.1:%d' % self.server.server_port

    def tearDown(self):
        self.server.shutdown(); self.server.server_close(); self.thread.join()

    def fetch(self, path, data=None, **headers):
        h = {'Content-Type': 'application/json', 'X-Console-Token': self.app.token}
        h.update(headers)
        req = urllib.request.Request(self.url + path, headers=h,
            data=None if data is None else json.dumps(data).encode())
        try:
            with urllib.request.urlopen(req) as r: return r.status, r.read()
        except urllib.error.HTTPError as e: return e.code, e.read()

    def test_no_hardware_contact_at_startup_or_status(self):
        with patch('web_app.request', side_effect=AssertionError('hardware contacted')):
            status, body = self.fetch('/api/status')
        self.assertEqual(status, 200)
        self.assertFalse(json.loads(body)['motion_available'])

    def test_csrf_host_origin_and_token(self):
        for headers in ({'Origin':'https://attacker.example'}, {'Host':'attacker.example'},
                        {'X-Console-Token':''}, {'Content-Type':'text/plain'}):
            with self.subTest(headers=headers):
                self.assertEqual(self.fetch('/api/stop', {}, **headers)[0], 403)
        self.assertEqual(self.fetch('/api/session', Host='attacker.example')[0], 403)
        self.assertEqual(self.fetch('/api/stop', {})[0], 200)

    def test_key_never_returned_and_disconnect_clears(self):
        with patch('web_app.call', return_value={'name':'models/gemini-4-argon'}):
            self.assertEqual(self.fetch('/api/connect', {'api_key':'unit-test-credential'})[0], 200)
        self.assertNotIn(b'unit-test-credential', self.fetch('/api/status')[1])
        self.assertEqual(self.fetch('/api/disconnect', {})[0], 200)
        self.assertEqual(self.app.key, '')

    def test_default_motion_locked_even_with_confirmations(self):
        self.app.key = 'unit-test-credential'; self.app.connected = True
        with patch('web_app.request', side_effect=AssertionError('hardware contacted')):
            code, body = self.fetch('/api/start', {'suspended':True, 'clearance':True})
        self.assertEqual(code, 400)
        self.assertIn(b'Motion is locked', body)

    def test_observe_sends_image_to_provider_without_robot(self):
        self.app.key = 'unit-test-credential'
        with patch.object(self.app.adapter, 'respond', return_value='A bottle. unit-test-credential') as model, \
             patch('web_app.request', side_effect=AssertionError('hardware contacted')):
            code, body = self.fetch('/api/observe', {'image':base64.b64encode(b'fixture').decode()})
        self.assertEqual(code, 200)
        self.assertFalse(json.loads(body)['motion_sent'])
        self.assertNotIn(b'unit-test-credential', body)
        self.assertEqual(model.call_args.kwargs['inputs'][1]['type'], 'image')

    def test_late_observation_discarded_after_stop(self):
        self.app.key = 'unit-test-credential'
        def delayed(**kwargs):
            self.app.stop()
            return 'stale proposal'
        with patch.object(self.app.adapter, 'respond', side_effect=delayed):
            code, body = self.fetch('/api/observe', {'image':base64.b64encode(b'fixture').decode()})
        self.assertEqual(code, 400)
        self.assertNotIn(b'stale proposal', body)
        self.assertFalse(self.app.observing)

    def test_failed_stop_is_not_acknowledged(self):
        self.app.connected = True
        with patch('web_app.request', side_effect=OSError):
            code, body = self.fetch('/api/stop', {})
        self.assertEqual(code, 200)
        self.assertFalse(json.loads(body)['robot_ack'])

    def test_non_object_unknown_path_and_traversal(self):
        self.assertEqual(self.fetch('/api/stop', [1])[0], 400)
        self.assertEqual(self.fetch('/api/nope', {})[0], 404)
        self.assertEqual(self.fetch('/../config/robot.json')[0], 404)
        self.assertEqual(self.fetch('/')[0], 200)

    def test_invalid_model_cannot_inject_url(self):
        with patch('web_app.call', side_effect=AssertionError('provider contacted')):
            self.assertEqual(self.fetch('/api/connect', {'api_key':'fixture', 'model':'../secret?key=x'})[0], 400)


class AdapterTests(unittest.TestCase):
    def test_no_hand_cartesian_or_other_body_actions(self):
        for op in ('open_hand', 'grasp_bottle', 'move_left_hand', 'place_at', 'release_at_B', 'exec'):
            with self.subTest(op=op), self.assertRaises(ValueError):
                validate_practice_action({'action':op})
        for joint in (True, 0, 21, 22, '15', 15.0):
            with self.subTest(joint=joint), self.assertRaises(ValueError):
                validate_practice_action({'action':'trial_left_arm', 'joint_id':joint})
        validate_practice_action({'action':'trial_left_arm', 'joint_id':15})

    def test_steps_response_and_incomplete_rejection(self):
        self.assertEqual(output_text({'status':'completed','steps':[{'type':'model_output','content':[{'type':'text','text':'scene'}]}]}), 'scene')
        with self.assertRaises(ValueError):output_text({'status':'in_progress','output_text':'unsafe'})

    def test_provider_error_body_never_exposed(self):
        error = urllib.error.HTTPError('https://example.com', 429, 'quota', {}, io.BytesIO(b'SECRET_REQUEST'))
        with patch('urllib.request.urlopen', side_effect=error), self.assertRaises(ValueError) as ctx:
            call('fixture', '/interactions', {})
        self.assertNotIn('SECRET', str(ctx.exception))
        self.assertIn('Quota', str(ctx.exception))

    def test_adapter_disables_storage_and_uses_expected_endpoint(self):
        with patch('agent_adapter.call', return_value={'status':'completed','output_text':'ok'}) as transport:
            self.assertEqual(GeminiAdapter().respond(key='fixture',model='gemini-4-argon',thinking='low',inputs=[]), 'ok')
        self.assertEqual(transport.call_args.args[1], '/interactions')
        self.assertIs(transport.call_args.args[2]['store'], False)

if __name__ == '__main__': unittest.main()

class GatewayStartupTests(unittest.TestCase):
    def test_observing_startup_does_not_release_an_existing_hold(self):
        # Execute the actual control-loop function with a single mocked tick.
        # No SDK import, network interface or hardware writes are involved.
        import ast
        from types import SimpleNamespace
        from unittest.mock import Mock
        source = Path(__file__).resolve().parents[1] / 'src/robot_stop_bridge.py'
        tree = ast.parse(source.read_text())
        function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'control_loop')
        module = ast.Module(body=[function], type_ignores=[])
        class EndTick(Exception): pass
        core = SimpleNamespace(stopped=True, active=False, tick=Mock())
        publisher = Mock()
        namespace = {'core':core, 'state':SimpleNamespace(motor_state=[SimpleNamespace(mode=1)] * 29),
            'lock':threading.RLock(), 'LEFT':range(15,22), 'last_loop':0,
            'arm_writes':0, 'last_fault':None, 'stopped_since':0, 'relax_requested_at':None,
            'time':SimpleNamespace(monotonic=lambda:1, sleep=Mock(side_effect=EndTick)), 'publisher':publisher}
        exec(compile(module, str(source), 'exec'), namespace)
        with self.assertRaises(EndTick):namespace['control_loop']()
        publisher.Write.assert_not_called()
        self.assertFalse(core.active)
        self.assertIsNone(namespace['relax_requested_at'])
