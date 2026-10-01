"""Check the measured route and stage order without requiring ROS."""

import ast
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
    namespace = {'rospy': type('Logger', (), {'loginfo': staticmethod(lambda *args: None)})}
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
        a = {'x': 3.068969249725342, 'y': 1.4840493202209473,
             'z': -0.7707573704168202, 'w': 0.6371287750118877}
        b = load_method('opposite_pose')(a)
        self.assertEqual((a['x'], a['y']), (b['x'], b['y']))
        self.assertAlmostEqual(a['z'] * b['z'] + a['w'] * b['w'], 0.0)

    def test_launch_uses_latest_measured_waypoints(self):
        root = ET.parse(LAUNCH).getroot()
        args = {item.attrib['name']: item.attrib['default']
                for item in root.findall('arg')}
        expected_poses = {
            'first_corner': (3.824002265930176, 0.5524806976318359,
                             0.7105965273343292, 0.7035997266488895),
            'next_corner': (3.624973773956299, 1.540183663368225,
                            -0.9999627453743581, 0.008631793751969744),
            'area_middle': (3.068969249725342, 1.4840493202209473,
                            -0.7707573704168202, 0.6371287750118877),
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


if __name__ == '__main__':
    unittest.main()
