"""Check the mission stage order without requiring a ROS installation."""

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

        mission = FakeMission()
        for name in ('first_corner', 'turn_after_first', 'next_corner',
                     'area_middle', 'area_a', 'area_b', 'second_corner',
                     'second_stop', 'second_cross'):
            setattr(mission, name + '_pose', name)

        load_method('run_two_intersections')(mission)
        self.assertEqual([event for event in events if event[0] == 'recognize'], [
            ('recognize', 'A 街区', 1), ('recognize', 'B 街区', 2)])
        self.assertEqual([event[1] for event in events if event[0] == 'navigate'], [
            '第一处路口后拐点', '第一次转向后路径点', '下一个拐弯点',
            '街区中间', 'A 街区人物观察点', 'B 街区人物观察点',
            '第二处路口前拐点', '第二处红绿灯停止线前', '第二处路口后目标点'])
        self.assertLess(events.index(('green', '第二处红绿灯')),
                        events.index(('navigate', '第二处路口后目标点')))

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

    def test_launch_requires_corrected_turn_pose(self):
        root = ET.parse(LAUNCH).getroot()
        args = {item.attrib['name']: item.attrib['default']
                for item in root.findall('arg')}
        for part in ('x', 'y', 'qz', 'qw'):
            self.assertEqual(args['turn_after_first_' + part], 'UNSET')
        self.assertEqual(args['second_cross_x'], '2.200')
        self.assertEqual(args['second_cross_y'], '3.750')


if __name__ == '__main__':
    unittest.main()
