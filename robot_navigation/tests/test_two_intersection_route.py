"""Check the measured route and stage order without requiring ROS."""

import ast
import math
import pathlib
import unittest
import xml.etree.ElementTree as ET


PACKAGE = pathlib.Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / 'scripts' / 'single_intersection_mission.py'
LAUNCH = PACKAGE / 'launch' / 'two_intersection_mission.launch'


def load_method(method_name):
    source = ast.parse(SCRIPT.read_text(encoding='utf-8'))
    mission_class = next(node for node in source.body
                         if isinstance(node, ast.ClassDef)
                         and node.name == 'SingleIntersectionMission')
    route = next(node for node in mission_class.body
                 if isinstance(node, ast.FunctionDef)
                 and node.name == method_name)
    namespace = {
        'math': math,
        'rospy': type('Logger', (), {
            'loginfo': staticmethod(lambda *args: None)})}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[route], type_ignores=[])),
                 str(SCRIPT), 'exec'), namespace)
    return namespace[method_name]


class RouteTest(unittest.TestCase):
    def test_stage_order_and_two_distinct_person_calls(self):
        events = []

        class FakeMission:
            stop_settle_time = 1.0

            def navigate(self, label, pose):
                events.append(('navigate', label))

            def hold_stopped(self, duration):
                events.append(('hold', duration))

            def recognize_people(self, area, flag):
                events.append(('recognize', area, flag))
                return {'community': 1, 'non-community': 0}

            def advance_before_turn(self):
                events.append(('advance', 0.05))
                self.area_b_pose = 'turn_prep'

            def wait_for_green(self, label):
                events.append(('green', label))

            def stop_robot(self):
                events.append(('stop',))

        mission = FakeMission()
        for name in ('first_corner', 'next_corner',
                     'area_middle', 'area_b', 'second_corner',
                     'second_stop', 'second_cross'):
            setattr(mission, name + '_pose', name)

        load_method('run_two_intersections')(mission)
        self.assertEqual([event for event in events if event[0] == 'recognize'], [
            ('recognize', 'A 街区', 1), ('recognize', 'B 街区', 2)])
        self.assertEqual([event[1] for event in events if event[0] == 'navigate'], [
            '第一处路口后拐点', '下一个拐弯点',
            '街区中间 A 街区观察点', '原地转向 B 街区',
            '第二处路口前拐点', '第二处红绿灯停止线前', '第二处路口后面向车牌点'])
        self.assertLess(events.index(('recognize', 'A 街区', 1)),
                        events.index(('advance', 0.05)))
        self.assertLess(events.index(('advance', 0.05)),
                        events.index(('navigate', '原地转向 B 街区')))
        self.assertLess(events.index(('navigate', '原地转向 B 街区')),
                        events.index(('recognize', 'B 街区', 2)))
        self.assertLess(events.index(('green', '第二处红绿灯')),
                        events.index(('navigate', '第二处路口后面向车牌点')))
        self.assertEqual(events[-1], ('stop',))

    def test_first_light_and_crossing_precede_new_route(self):
        events = []

        class FakeMission:
            stop_pose = 'stop'
            cross_pose = 'cross'
            stop_settle_time = 1.0
            two_intersection_enabled = True
            two_area_enabled = False

            def wait_for_action_server(self):
                events.append('server')

            def navigate(self, label, pose):
                events.append(('navigate', label))

            def hold_stopped(self, duration):
                events.append('hold')

            def wait_for_green(self, label):
                events.append(('green', label))

            def run_two_intersections(self):
                events.append('new_route')

        load_method('run')(FakeMission())
        self.assertEqual(events, [
            'server', ('navigate', '停止线前观察点'), 'hold',
            ('green', '第一处红绿灯'), ('navigate', '路口后目标点'),
            'hold', 'new_route'])

    def test_b_observation_rotates_at_same_position(self):
        a = {'x': 2.900, 'y': 1.700,
             'z': -0.701, 'w': 0.713}
        b = load_method('opposite_pose')(a)
        self.assertEqual((a['x'], a['y']), (b['x'], b['y']))
        self.assertAlmostEqual(a['z'] * b['z'] + a['w'] * b['w'], 0.0)

    def test_turn_prep_goal_is_five_centimeters_forward(self):
        a = {'x': 2.900, 'y': 1.700, 'z': -0.701, 'w': 0.713}
        target = load_method('forward_pose')(a, 0.05)
        self.assertAlmostEqual(math.hypot(target['x'] - a['x'],
                                          target['y'] - a['y']), 0.05)
        self.assertLess(target['y'], a['y'])
        self.assertEqual((target['z'], target['w']), (a['z'], a['w']))

    def test_short_advance_uses_actual_displacement_and_stops(self):
        position = {'x': 2.900, 'y': 1.700}
        commands = []

        class Twist:
            def __init__(self):
                self.linear = type('Linear', (), {'x': 0.0})()

        class Publisher:
            def publish(self, command):
                commands.append(command.linear.x)
                if command.linear.x > 0:
                    position['y'] -= 0.01

        class Mission:
            turn_prep_distance = 0.05
            area_middle_pose = {'x': 2.900, 'y': 1.700,
                                'z': -0.701, 'w': 0.713}
            velocity = Publisher()
            opposite_pose = staticmethod(load_method('opposite_pose'))
            forward_pose = staticmethod(load_method('forward_pose'))

            @staticmethod
            def current_map_pose():
                return {'x': position['x'], 'y': position['y'],
                        'z': -0.701, 'w': 0.713}

        advance = load_method('advance_before_turn')
        advance.__globals__.update({
            'Twist': Twist, 'MissionError': RuntimeError,
            'rospy': type('Ros', (), {
                'is_shutdown': staticmethod(lambda: False),
                'loginfo': staticmethod(lambda *args: None)}),
            'time': type('Clock', (), {
                'monotonic': staticmethod(lambda: 0.0),
                'sleep': staticmethod(lambda duration: None)})})
        mission = Mission()
        advance(mission)
        self.assertAlmostEqual(position['y'], 1.650)
        self.assertEqual(commands[-1], 0.0)
        self.assertTrue(all(speed > 0 for speed in commands[:-1]))
        self.assertAlmostEqual(mission.area_b_pose['y'], 1.650)

    def test_launch_uses_current_route_waypoints(self):
        root = ET.parse(LAUNCH).getroot()
        args = {item.attrib['name']: item.attrib['default']
                for item in root.findall('arg')}
        self.assertAlmostEqual(float(args['turn_prep_distance']), 0.05)
        expected_poses = {
            'first_corner': (3.824002265930176, 0.5524806976318359,
                             0.7105965273343292, 0.7035997266488895),
            'next_corner': (3.566, 1.579,
                            -0.9999627453743581, 0.008631793751969744),
            'area_middle': (2.900, 1.700, -0.701, 0.713),
            'second_corner': (2.0103862285614014, 1.41770339012146,
                              0.7093058728328143, 0.7049008290283673),
            'second_stop': (2.132756233215332, 2.3429512977600098,
                            0.7130028121783379, 0.701161172503),
            'second_cross': (2.1445207595825195, 3.8558530807495117,
                             0.9999972397584217, 0.002349569223860622),
        }
        for name, pose in expected_poses.items():
            for part, expected in zip(('x', 'y', 'qz', 'qw'), pose):
                self.assertAlmostEqual(float(args[name + '_' + part]), expected)
        self.assertFalse(any(name.startswith(('turn_after_first_', 'area_a_',
                                              'area_b_')) for name in args))

    def test_next_corner_shift_is_forward_and_left_of_approach(self):
        first = (3.824002265930176, 0.5524806976318359)
        original = (3.624973773956299, 1.540183663368225)
        adjusted = (3.566, 1.579)
        dx = original[0] - first[0]
        dy = original[1] - first[1]
        distance = math.hypot(dx, dy)
        forward = (dx / distance, dy / distance)
        left = (-forward[1], forward[0])
        shift = (adjusted[0] - original[0], adjusted[1] - original[1])
        self.assertAlmostEqual(sum(a * b for a, b in zip(shift, forward)),
                               0.05, delta=0.001)
        self.assertAlmostEqual(sum(a * b for a, b in zip(shift, left)),
                               0.05, delta=0.001)


if __name__ == '__main__':
    unittest.main()
