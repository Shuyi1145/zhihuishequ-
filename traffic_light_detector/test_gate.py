"""Run with PYTHONPATH=src python -m unittest discover -p test_gate.py."""
import unittest
from traffic_light_detector.gate import GreenGate


class GateTests(unittest.TestCase):
    def test_green_needs_duration_and_frames(self):
        gate = GreenGate()
        gate.update('green', 10)
        gate.update('green', 10.2)
        self.assertEqual(gate.snapshot(10.2), ('green', False))
        gate.update('green', 10.6)
        self.assertEqual(gate.snapshot(10.6), ('green', True))
        self.assertEqual(gate.snapshot(11.7), ('unknown', False))

    def test_red_unknown_and_yellow_cancel_green(self):
        for state in ('red', 'yellow', 'unknown'):
            gate = GreenGate()
            for stamp in (1, 1.3, 1.6):
                gate.update('green', stamp)
            gate.update(state, 1.7)
            self.assertFalse(gate.snapshot(1.7)[1])

    def test_duplicate_and_gap_do_not_confirm(self):
        gate = GreenGate()
        for _ in range(10):
            gate.update('green', 1)
        self.assertFalse(gate.snapshot(1)[1])
        gate.update('green', 3)
        self.assertFalse(gate.snapshot(3)[1])

    def test_reset_for_new_junction(self):
        gate = GreenGate()
        for stamp in (1, 1.3, 1.6):
            gate.update('green', stamp)
        gate.reset()
        self.assertEqual(gate.snapshot(1.7), ('unknown', False))


if __name__ == '__main__':
    unittest.main()
