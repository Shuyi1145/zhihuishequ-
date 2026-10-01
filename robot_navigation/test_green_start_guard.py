"""Test the mission's pure timing guard without requiring a ROS installation."""

import ast
import math
from pathlib import Path
import unittest


def load_guard():
    source = Path(__file__).with_name('scripts').joinpath(
        'single_intersection_mission.py')
    tree = ast.parse(source.read_text(encoding='utf-8'), filename=str(source))
    guard_class = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'GreenStartGuard'
    )
    namespace = {'math': math}
    module = ast.Module(body=[guard_class], type_ignores=[])
    exec(compile(module, str(source), 'exec'), namespace)
    return namespace['GreenStartGuard']


GreenStartGuard = load_guard()


class GreenStartGuardTests(unittest.TestCase):
    def test_green_already_on_at_arrival_is_not_enough(self):
        guard = GreenStartGuard(2.0)
        self.assertFalse(guard.observe('green', True, 10.0))
        self.assertFalse(guard.observe('unknown', False, 10.2))
        self.assertFalse(guard.observe('green', True, 10.5))

    def test_reported_late_green_waits_for_next_cycle(self):
        guard = GreenStartGuard(2.0)
        self.assertFalse(guard.observe('unknown', False, 11933.429))
        self.assertFalse(guard.observe('green', False, 11933.518))
        self.assertFalse(guard.observe('green', True, 11933.778))
        self.assertFalse(guard.observe('yellow', False, 11933.893))
        guard.observe('red', False, 11943.9)
        self.assertTrue(guard.observe('green', True, 11944.8))

    def test_recent_red_then_confirmed_green_allows_departure(self):
        guard = GreenStartGuard(2.0)
        self.assertFalse(guard.observe('red', False, 10.0))
        self.assertFalse(guard.observe('green', False, 10.4))
        self.assertTrue(guard.observe('green', True, 10.8))

    def test_late_green_and_yellow_are_rejected(self):
        guard = GreenStartGuard(2.0)
        guard.observe('red', False, 10.0)
        self.assertFalse(guard.observe('green', True, 12.1))
        guard.observe('yellow', False, 12.2)
        self.assertFalse(guard.observe('green', True, 12.3))

    def test_short_unknown_dropout_does_not_invent_a_new_red(self):
        guard = GreenStartGuard(2.0)
        guard.observe('red', False, 10.0)
        guard.observe('unknown', False, 10.3)
        self.assertTrue(guard.observe('green', True, 10.8))
        self.assertFalse(guard.observe('green', True, 12.1))

    def test_simulation_clock_rewind_forgets_old_cycle(self):
        guard = GreenStartGuard(2.0)
        guard.observe('red', False, 10.0)
        self.assertFalse(guard.observe('green', True, 1.0))
        self.assertFalse(guard.observe('green', True, math.nan))


if __name__ == '__main__':
    unittest.main()
