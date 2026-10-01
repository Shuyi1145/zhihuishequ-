#!/usr/bin/env python3
"""Run one or two intersections with the existing signal and person services."""

import json
import math
import sys
import time

import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
import tf2_ros
from robot_navigation.srv import detect
from std_srvs.srv import Trigger


class MissionError(RuntimeError):
    pass


class SingleIntersectionMission:
    def __init__(self):
        self.move_base_name = rospy.get_param('~move_base_action', '/move_base')
        self.check_service_name = rospy.get_param('~check_service', '/traffic_light/check')
        self.reset_service_name = rospy.get_param('~reset_service', '/traffic_light/reset')
        self.goal_timeout = float(rospy.get_param('~goal_timeout', 60.0))
        self.green_timeout = float(rospy.get_param('~green_timeout', 120.0))
        self.poll_period = float(rospy.get_param('~poll_period', 0.2))
        self.stop_settle_time = float(rospy.get_param('~stop_settle_time', 1.0))
        self.turn_prep_distance = float(rospy.get_param('~turn_prep_distance', 0.05))
        self.stop_pose = self.read_pose('stop')
        self.cross_pose = self.read_pose('cross')
        self.two_area_enabled = rospy.get_param('~two_area_enabled', False)
        self.two_intersection_enabled = rospy.get_param('~two_intersection_enabled', False)
        if self.two_area_enabled and self.two_intersection_enabled:
            raise MissionError('two_area_enabled 与 two_intersection_enabled 不能同时启用')
        if self.two_intersection_enabled:
            self.first_corner_pose = self.read_pose('first_corner')
            self.next_corner_pose = self.read_pose('next_corner')
            self.area_middle_pose = self.read_pose('area_middle')
            self.area_b_pose = self.opposite_pose(self.area_middle_pose)
            self.second_corner_pose = self.read_pose('second_corner')
            self.second_stop_pose = self.read_pose('second_stop')
            self.second_cross_pose = self.read_pose('second_cross')
        elif self.two_area_enabled:
            self.area_a_pose = self.read_pose('area_a')
            self.area_b_pose = self.read_pose('area_b')
            self.second_corner_pose = self.read_pose('second_corner')
            self.second_stop_pose = self.read_pose('second_stop')
        self.person_service_name = rospy.get_param('~person_service', '/recognize_person')
        self.validate()

        self.velocity = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.client = actionlib.SimpleActionClient(self.move_base_name, MoveBaseAction)
        if self.two_intersection_enabled:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.on_shutdown(self.stop_robot)

    @staticmethod
    def read_pose(prefix):
        try:
            return {
                'x': float(rospy.get_param('~{}_x'.format(prefix))),
                'y': float(rospy.get_param('~{}_y'.format(prefix))),
                'z': float(rospy.get_param('~{}_qz'.format(prefix))),
                'w': float(rospy.get_param('~{}_qw'.format(prefix))),
            }
        except (KeyError, TypeError, ValueError) as exc:
            raise MissionError('{} 点坐标未设置或格式错误'.format(prefix)) from exc

    @staticmethod
    def opposite_pose(pose):
        """Keep the observation position and rotate its yaw by 180 degrees."""
        return {'x': pose['x'], 'y': pose['y'],
                'z': pose['w'], 'w': -pose['z']}

    def validate(self):
        for name, value in (
                ('goal_timeout', self.goal_timeout),
                ('green_timeout', self.green_timeout),
                ('poll_period', self.poll_period),
                ('stop_settle_time', self.stop_settle_time),
                ('turn_prep_distance', self.turn_prep_distance)):
            if value <= 0:
                raise MissionError('{} must be positive'.format(name))
        poses = [('stop', self.stop_pose), ('cross', self.cross_pose)]
        if self.two_intersection_enabled:
            poses.extend((('first_corner', self.first_corner_pose),
                          ('next_corner', self.next_corner_pose),
                          ('area_middle', self.area_middle_pose),
                          ('area_b', self.area_b_pose),
                          ('second_corner', self.second_corner_pose),
                          ('second_stop', self.second_stop_pose),
                          ('second_cross', self.second_cross_pose)))
        elif self.two_area_enabled:
            poses.extend((('area_a', self.area_a_pose),
                          ('area_b', self.area_b_pose),
                          ('second_corner', self.second_corner_pose),
                          ('second_stop', self.second_stop_pose)))
        for name, pose in poses:
            if not all(math.isfinite(value) for value in pose.values()):
                raise MissionError('{} pose contains a non-finite value'.format(name))
            norm = math.hypot(pose['z'], pose['w'])
            if norm < 1e-6:
                raise MissionError('{} orientation quaternion is invalid'.format(name))
            pose['z'] /= norm
            pose['w'] /= norm

    def stop_robot(self):
        if hasattr(self, 'client'):
            self.client.cancel_all_goals()
        if hasattr(self, 'velocity'):
            zero = Twist()
            # Several messages make the fail-safe stop visible to the drive plugin.
            for _ in range(3):
                self.velocity.publish(zero)
                time.sleep(0.05)

    def wait_for_action_server(self):
        deadline = time.monotonic() + 15.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.client.wait_for_server(rospy.Duration(0.2)):
                return
        raise MissionError('move_base action server is unavailable')

    def goal(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = pose['x']
        goal.target_pose.pose.position.y = pose['y']
        goal.target_pose.pose.orientation.z = pose['z']
        goal.target_pose.pose.orientation.w = pose['w']
        return goal

    def navigate(self, label, pose):
        rospy.loginfo('导航到%s: x=%.3f, y=%.3f', label, pose['x'], pose['y'])
        self.client.send_goal(self.goal(pose))
        deadline = time.monotonic() + self.goal_timeout
        terminal_states = {
            GoalStatus.PREEMPTED,
            GoalStatus.SUCCEEDED,
            GoalStatus.ABORTED,
            GoalStatus.REJECTED,
            GoalStatus.RECALLED,
            GoalStatus.LOST,
        }
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            state = self.client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo('已到达%s', label)
                return
            if state in terminal_states:
                raise MissionError('导航到{}失败，move_base 状态={}'.format(label, state))
            time.sleep(0.1)
        self.client.cancel_goal()
        raise MissionError('导航到{}超时'.format(label))

    @staticmethod
    def wait_for_service_wall(name, timeout):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                rospy.wait_for_service(name, timeout=0.2)
                return
            except rospy.ROSException:
                pass
        raise MissionError('服务不可用: {}'.format(name))

    def hold_stopped(self, duration):
        deadline = time.monotonic() + duration
        zero = Twist()
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.velocity.publish(zero)
            time.sleep(min(self.poll_period, 0.1))

    def wait_for_green(self, label):
        self.wait_for_service_wall(self.reset_service_name, 10.0)
        self.wait_for_service_wall(self.check_service_name, 10.0)
        reset = rospy.ServiceProxy(self.reset_service_name, Trigger)
        check = rospy.ServiceProxy(self.check_service_name, Trigger)
        response = reset()
        if not response.success:
            raise MissionError('红绿灯状态重置失败: {}'.format(response.message))

        rospy.loginfo('已停在%s停止线前，开始等待绿灯', label)
        deadline = time.monotonic() + self.green_timeout
        last_state = None
        zero = Twist()
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.velocity.publish(zero)
            try:
                response = check()
                data = json.loads(response.message)
                state = data.get('state', 'unknown')
                allowed = response.success and data.get('allowed') is True and state == 'green'
                if state != last_state:
                    rospy.loginfo('当前灯色=%s，可通行=%s', state, allowed)
                    last_state = state
                if allowed:
                    rospy.loginfo('%s绿灯连续确认通过', label)
                    return
            except (rospy.ServiceException, ValueError, TypeError) as exc:
                rospy.logwarn_throttle(5.0, '红绿灯查询失败，继续停车: %s', exc)
            time.sleep(self.poll_period)
        raise MissionError('等待绿灯超时，保持停车')

    @staticmethod
    def parse_person_counts(result):
        if result.startswith('ERROR:'):
            raise MissionError('人物识别失败: {}'.format(result))
        counts = {'community': 0, 'non-community': 0}
        if not result.strip():
            return counts
        for item in result.split(','):
            name, separator, value = item.strip().partition(':')
            if not separator or name not in counts or not value.isdigit():
                raise MissionError('人物识别返回格式错误: {}'.format(result))
            counts[name] = int(value)
        return counts

    def recognize_people(self, area_name, detect_flag):
        self.wait_for_service_wall(self.person_service_name, 10.0)
        service = rospy.ServiceProxy(self.person_service_name, detect)
        response = service(detect_flag)
        counts = self.parse_person_counts(response.result)
        if not any(counts.values()):
            raise MissionError('{}未识别到人员，不能把空结果当作已完成识别'.format(area_name))
        rospy.loginfo('%s识别：community=%d, non-community=%d',
                      area_name, counts['community'], counts['non-community'])
        return counts

    def current_map_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            raise MissionError('无法读取转向前车体位置: {}'.format(exc)) from exc
        position = transform.transform.translation
        rotation = transform.transform.rotation
        pose = {'x': position.x, 'y': position.y,
                'z': rotation.z, 'w': rotation.w}
        if (not all(math.isfinite(value) for value in pose.values())
                or math.hypot(pose['z'], pose['w']) < 1e-6):
            raise MissionError('转向前车体 TF 位姿无效')
        return pose

    @staticmethod
    def forward_pose(pose, distance):
        yaw = 2.0 * math.atan2(pose['z'], pose['w'])
        return {'x': pose['x'] + distance * math.cos(yaw),
                'y': pose['y'] + distance * math.sin(yaw),
                'z': pose['z'], 'w': pose['w']}

    def advance_before_turn(self):
        """Move a measured short distance; move_base's 15 cm tolerance would skip it."""
        start = self.current_map_pose()
        target = self.forward_pose(start, self.turn_prep_distance)
        heading = 2.0 * math.atan2(start['z'], start['w'])
        rospy.loginfo('A 识别后向前 %.3f m，转向准备点: x=%.3f, y=%.3f',
                      self.turn_prep_distance, target['x'], target['y'])
        deadline = time.monotonic() + 6.0
        try:
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                actual = self.current_map_pose()
                dx = actual['x'] - start['x']
                dy = actual['y'] - start['y']
                forward = dx * math.cos(heading) + dy * math.sin(heading)
                sideways = abs(-dx * math.sin(heading) + dy * math.cos(heading))
                if sideways > 0.03 or forward < -0.01:
                    raise MissionError('转向准备点前进时发生明显偏移')
                if forward >= self.turn_prep_distance - 0.005:
                    self.area_b_pose = self.opposite_pose(self.area_middle_pose)
                    self.area_b_pose['x'] = actual['x']
                    self.area_b_pose['y'] = actual['y']
                    rospy.loginfo('已到转向准备点: x=%.3f, y=%.3f', actual['x'], actual['y'])
                    return
                command = Twist()
                command.linear.x = min(0.05, max(0.02, 0.8 * (self.turn_prep_distance - forward)))
                self.velocity.publish(command)
                time.sleep(0.05)
            raise MissionError('转向准备点前进超时或 ROS 已关闭')
        finally:
            self.velocity.publish(Twist())

    def run_two_intersections(self):
        for label, pose in (
                ('第一处路口后拐点', self.first_corner_pose),
                ('下一个拐弯点', self.next_corner_pose),
                ('街区中间 A 街区观察点', self.area_middle_pose)):
            self.navigate(label, pose)

        self.hold_stopped(self.stop_settle_time)
        area_a_counts = self.recognize_people('A 街区', 1)
        self.hold_stopped(self.stop_settle_time)

        self.advance_before_turn()
        self.hold_stopped(self.stop_settle_time)
        self.navigate('原地转向 B 街区', self.area_b_pose)
        self.hold_stopped(self.stop_settle_time)
        area_b_counts = self.recognize_people('B 街区', 2)
        self.hold_stopped(self.stop_settle_time)
        rospy.loginfo('两街区分别完成一次识别：A=%s，B=%s；视野重叠前不直接相加',
                      area_a_counts, area_b_counts)

        self.navigate('第二处路口前拐点', self.second_corner_pose)
        self.navigate('第二处红绿灯停止线前', self.second_stop_pose)
        self.hold_stopped(self.stop_settle_time)
        self.wait_for_green('第二处红绿灯')
        self.navigate('第二处路口后面向车牌点', self.second_cross_pose)
        self.hold_stopped(self.stop_settle_time)
        self.stop_robot()
        rospy.loginfo('已到达第二处路口后面向车牌并停车；本阶段结束，未执行车牌识别')

    def run(self):
        self.wait_for_action_server()
        self.navigate('停止线前观察点', self.stop_pose)
        self.hold_stopped(self.stop_settle_time)
        self.wait_for_green('第一处红绿灯')
        self.navigate('路口后目标点', self.cross_pose)
        self.hold_stopped(self.stop_settle_time)
        if self.two_intersection_enabled:
            self.run_two_intersections()
        elif self.two_area_enabled:
            self.navigate('A 街区人物观察点', self.area_a_pose)
            self.hold_stopped(self.stop_settle_time)
            area_a_counts = self.recognize_people('A 街区', 1)
            self.hold_stopped(self.stop_settle_time)
            self.navigate('B 街区人物观察点', self.area_b_pose)
            self.hold_stopped(self.stop_settle_time)
            area_b_counts = self.recognize_people('B 街区', 2)
            self.hold_stopped(self.stop_settle_time)
            rospy.loginfo('两街区分别完成一次识别：A=%s，B=%s；视野重叠前不直接相加',
                          area_a_counts, area_b_counts)
            self.navigate('B 街区后拐角', self.second_corner_pose)
            self.navigate('第二处红绿灯停止线前', self.second_stop_pose)
            self.hold_stopped(self.stop_settle_time)
            self.wait_for_green('第二处红绿灯')
            self.hold_stopped(self.stop_settle_time)
            rospy.loginfo('已确认第二处绿灯；本阶段停在第二处停止线前，不越过路口')
        else:
            rospy.loginfo('单路口任务完成')


def main():
    rospy.init_node('single_intersection_mission')
    mission = None
    try:
        mission = SingleIntersectionMission()
        mission.run()
        return 0
    except (MissionError, rospy.ROSException, rospy.ServiceException) as exc:
        rospy.logerr('路口任务失败: %s', exc)
        if mission is not None:
            mission.stop_robot()
        return 1


if __name__ == '__main__':
    sys.exit(main())
