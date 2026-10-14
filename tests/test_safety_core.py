import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from safety_core import SafetyCore


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.core = SafetyCore(lambda: self.now)
        self.core.observe()
        self.core.heartbeat()
        self.core.enable([0.0] * 7, [0.8] * 6)

    def test_stop_freezes_arm_and_grip_and_rejects_queued_move(self):
        c = self.core
        generation = c.generation
        c.move(18, 0.03, 0.01, generation)
        c.hand_move([0.7] * 6, generation)
        c.tick(0.01)
        q, hand = c.q.copy(), c.hand_q.copy()
        c.stop()
        for _ in range(50):
            c.tick(0.01)
        self.assertEqual(c.q, q)
        self.assertEqual(c.hand_q, hand)
        with self.assertRaises(ValueError):
            c.move(18, 0.02, 0.01, generation)

    def test_pc_disconnection_stops_locally(self):
        self.now += 0.41
        self.core.observe()
        self.core.tick(0.004)
        self.assertTrue(self.core.stopped)
        self.assertEqual(self.core.reason, "PC heartbeat timeout")

    def test_state_loss_stops_even_with_pc_heartbeat(self):
        self.now += 0.11
        self.core.heartbeat()
        self.core.tick(0.004)
        self.assertTrue(self.core.stopped)
        self.assertEqual(self.core.reason, "Robot state timeout")

    def test_wrong_side_nonfinite_and_excess_motion_rejected(self):
        for joint, delta, speed in [(22, .01, .01), (14, .01, .01),
                                    (18, float('nan'), .01), (18, .031, .01),
                                    (18, .01, .02)]:
            with self.assertRaises(ValueError):
                self.core.move(joint, delta, speed, self.core.generation)

    def test_repeated_stop_invalidates_start_race(self):
        self.core.stop()
        before = self.core.generation
        self.core.stop()
        self.assertGreater(self.core.generation, before)

    def test_timing_fault_freezes(self):
        self.core.tick(0.06)
        self.assertTrue(self.core.stopped)

    def test_start_without_live_viewer_rejected(self):
        self.core.stop()
        self.now += 0.5
        self.core.observe()
        with self.assertRaises(ValueError):
            self.core.enable([0.0] * 7)


if __name__ == "__main__":
    unittest.main()
