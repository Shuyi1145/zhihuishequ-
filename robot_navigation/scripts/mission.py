#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
import tf
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger, TriggerRequest

from robot_navigation.srv import detect, detectRequest


class MissionController(object):
    def __init__(self):
        rospy.init_node("mission", anonymous=False)
        rospy.on_shutdown(self.shutdown)

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.navigation_index = -1
        self.navigation_status = "IDLE"
        self.current_yaw = None
        self.current_position = None
        self.waypoint_frame = "map"
        self.tf_listener = tf.TransformListener()
        self.waypoint_positions = []
        self.waypoint_path_poses = []
        # 配置使用界面显示的 1 起始 ID，内部触发时转换为 0 起始索引。
        # 这些列表允许通过 ROS 参数覆盖，便于更换路线时不改动任务流程。
        self.traffic_light_waypoints = self._get_waypoint_ids(
            "~traffic_light_waypoints", [1])
        self.person_waypoints = self._get_waypoint_ids(
            "~person_waypoints", [2, 4, 6, 8])
        self.plate_waypoints = self._get_waypoint_ids(
            "~plate_waypoints", [10, 11, 12])
        self.completed_traffic_light_waypoints = set()
        self.completed_person_waypoints = set()
        self.completed_plate_waypoints = set()
        self.total_person_count = 0
        self.street_person_counts = {"A": 0, "B": 0}
        self.street_non_community_counts = {"A": 0, "B": 0}

        # 在航点坐标系中使用平面 yaw 做闭环绝对朝向控制。
        self.turn_angular_speed = rospy.get_param("~turn_angular_speed", 0.6)
        self.turn_min_angular_speed = rospy.get_param("~turn_min_angular_speed", 0.18)
        self.turn_final_angular_speed = rospy.get_param("~turn_final_angular_speed", 0.06)
        self.turn_kp = rospy.get_param("~turn_kp", 1.8)
        self.turn_slowdown_deg = rospy.get_param("~turn_slowdown_deg", 20.0)
        self.turn_tolerance_deg = rospy.get_param("~turn_tolerance_deg", 2.0)
        self.turn_settle_time = rospy.get_param("~turn_settle_time", 0.5)
        self.turn_verify_timeout = rospy.get_param("~turn_verify_timeout", 2.0)
        self.task_waypoint_tolerance = rospy.get_param("~task_waypoint_tolerance", 0.1)

        self.start_navigation_srv = rospy.ServiceProxy(
            "/waypoint_editor/start_navigation", Trigger)
        self.pause_navigation_srv = rospy.ServiceProxy(
            "/waypoint_editor/pause_navigation", Trigger)
        self.resume_navigation_srv = rospy.ServiceProxy(
            "/waypoint_editor/resume_navigation", Trigger)
        self.stop_navigation_srv = rospy.ServiceProxy(
            "/waypoint_editor/stop_navigation", Trigger)
        self.person_client = rospy.ServiceProxy("/recognize_person", detect)
        self.plate_client = rospy.ServiceProxy("/recognize_plate", detect)
        self.traffic_light_client = rospy.ServiceProxy(
            "/recognize_traffic_light", detect)

        rospy.loginfo("等待航点导航服务")
        rospy.wait_for_service("/waypoint_editor/start_navigation")
        rospy.wait_for_service("/waypoint_editor/pause_navigation")
        rospy.wait_for_service("/waypoint_editor/resume_navigation")
        rospy.wait_for_service("/waypoint_editor/stop_navigation")

        rospy.loginfo("等待视觉识别服务")
        rospy.wait_for_service("/recognize_person")
        rospy.wait_for_service("/recognize_plate")
        rospy.wait_for_service("/recognize_traffic_light")

        rospy.Subscriber(
            "/waypoint_editor/navigation_index",
            Int32,
            lambda msg: setattr(self, "navigation_index", msg.data),
            queue_size=10)
        rospy.Subscriber(
            "/waypoint_editor/navigation_status",
            String,
            lambda msg: setattr(self, "navigation_status", msg.data.strip().upper()),
            queue_size=10)
        rospy.Subscriber(
            "/waypoint_editor/waypoints_path",
            Path,
            self.waypoints_callback,
            queue_size=1)
        rospy.Subscriber(
            rospy.get_param("~odom_topic", "/odom"),
            Odometry,
            self.odom_callback,
            queue_size=10)

        rospy.loginfo("任务控制器已就绪")

    def _get_waypoint_ids(self, param_name, default):
        """读取 1 起始航点 ID，转换成内部 0 起始索引并去重。"""
        configured = rospy.get_param(param_name, default)
        if not isinstance(configured, (list, tuple)):
            configured = default

        waypoint_ids = []
        for waypoint_id in configured:
            try:
                waypoint_id = int(waypoint_id)
            except (TypeError, ValueError):
                continue
            waypoint_index = waypoint_id - 1
            if waypoint_index >= 0 and waypoint_index not in waypoint_ids:
                waypoint_ids.append(waypoint_index)
        return waypoint_ids

    def odom_callback(self, msg):
        # 航点在 map 中，不能直接拿 odom 坐标参与 0.1 m 距离判断。
        pose = PoseStamped()
        pose.header.frame_id = msg.header.frame_id
        pose.header.stamp = rospy.Time(0)
        pose.pose = msg.pose.pose
        if not pose.header.frame_id:
            self.current_position = None
            rospy.logwarn_throttle(5.0, "里程计坐标系无效，暂不执行识别")
            return
        if pose.header.frame_id != self.waypoint_frame:
            try:
                pose = self.tf_listener.transformPose(self.waypoint_frame, pose)
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException) as exc:
                self.current_position = None
                return
        position = pose.pose.position
        self.current_position = (position.x, position.y)

        q = pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def waypoints_callback(self, msg):
        waypoint_frame = msg.header.frame_id or "map"
        if waypoint_frame != self.waypoint_frame:
            self.current_position = None
            self.waypoint_frame = waypoint_frame
        self.waypoint_path_poses = list(msg.poses)
        self.waypoint_positions = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
        ]

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(max_value, value))

    def wait_for_yaw(self):
        rate = rospy.Rate(20)
        wait_count = 0
        while not rospy.is_shutdown() and self.current_yaw is None and wait_count < 60:
            rate.sleep()
            wait_count += 1
        return self.current_yaw is not None

    def wait_for_target_yaw(self, target_yaw_deg):
        """确认里程计中的实际朝向已达到目标角度。"""
        if target_yaw_deg is None or not self.wait_for_yaw():
            return False

        target_yaw = math.radians(
            self.normalize_degrees(float(target_yaw_deg)))
        tolerance = math.radians(getattr(self, "turn_tolerance_deg", 2.0))
        verify_timeout = max(0.0, float(
            getattr(self, "turn_verify_timeout", 2.0)))
        rate_hz = 20.0
        max_wait_count = max(1, int(math.ceil(verify_timeout * rate_hz)))

        if (abs(self.normalize_angle(target_yaw - self.current_yaw))
                <= tolerance):
            return True

        rate = rospy.Rate(rate_hz)

        for _ in range(max_wait_count + 1):
            if (self.current_yaw is not None and
                    abs(self.normalize_angle(target_yaw - self.current_yaw))
                    <= tolerance):
                return True
            if rospy.is_shutdown():
                break
            rate.sleep()

        current_yaw_deg = (
            "unknown" if self.current_yaw is None else
            "%.1f" % math.degrees(self.current_yaw))
        rospy.logwarn(
            "等待机器人恢复到目标朝向 %.1f 度超时，当前朝向为 %s 度",
            math.degrees(target_yaw), current_yaw_deg)
        return False

    def normalize_degrees(self, angle_deg):
        while angle_deg > 180:
            angle_deg -= 360
        while angle_deg <= -180:
            angle_deg += 360
        return angle_deg

    def offset_yaw_deg(self, yaw_deg, offset_deg):
        """返回归一化后的绝对观察方向。"""
        if yaw_deg is None:
            return None
        return self.normalize_degrees(yaw_deg + offset_deg)

    def snap_axis_yaw_deg(self, yaw_deg):
        yaw_deg = self.normalize_degrees(yaw_deg)
        axis_candidates = [0, 90, 180, -90]
        return min(
            axis_candidates,
            key=lambda candidate: abs(self.normalize_degrees(yaw_deg - candidate)))

    def get_waypoint_yaw_deg(self, waypoint_index):
        if (waypoint_index < 0 or
                waypoint_index >= len(self.waypoint_positions) or
                waypoint_index >= len(self.waypoint_path_poses)):
            rospy.logerr("航点 %d 缺少姿态数据", waypoint_index + 1)
            return None

        waypoint_path_pose = self.waypoint_path_poses[waypoint_index]
        q = waypoint_path_pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        raw_yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        return int(self.snap_axis_yaw_deg(raw_yaw_deg))

    def rotate_to_yaw(self, target_yaw_deg):
        if target_yaw_deg is None:
            return

        if not self.wait_for_yaw():
            return

        max_speed = abs(self.turn_angular_speed)
        min_speed = abs(self.turn_min_angular_speed)
        final_min_speed = abs(self.turn_final_angular_speed)
        if max_speed < 1e-3:
            return

        target_yaw_deg = int(round(self.normalize_degrees(target_yaw_deg)))
        target_yaw = math.radians(target_yaw_deg)
        tolerance = math.radians(self.turn_tolerance_deg)
        slowdown_error = math.radians(self.turn_slowdown_deg)
        

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            error = self.normalize_angle(target_yaw - self.current_yaw)
            if abs(error) <= tolerance:
                break

            current_max_speed = max_speed
            current_min_speed = min_speed
            if abs(error) <= slowdown_error:
                current_max_speed = max(final_min_speed, max_speed * 0.35)
                current_min_speed = final_min_speed

            angular_z = self.clamp(self.turn_kp * error, -current_max_speed, current_max_speed)
            if abs(angular_z) < current_min_speed and abs(error) > tolerance * 2.0:
                angular_z = current_min_speed if error > 0 else -current_min_speed

            twist = Twist()
            twist.angular.z = angular_z
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(self.turn_settle_time)
   

    def pause_navigation(self):
        # 最后一个航点可能已经让 waypoint_editor 发布 FINISHED；此时没有
        # 可取消的导航目标，但执行识别任务仍然是安全的。
        if self.navigation_status in ("PAUSED", "FINISHED"):
            return True
        if self.navigation_status != "FOLLOWING":
            return False

        try:
            pause_result = self.pause_navigation_srv(TriggerRequest())
        except rospy.ServiceException as exc:
            rospy.logerr("暂停导航服务调用失败：%s", exc)
            return False
        if not pause_result.success:
            rospy.logerr("暂停导航失败：%s", pause_result.message)
            return False

        wait_count = 0
        while not rospy.is_shutdown() and self.navigation_status != "PAUSED" and wait_count < 50:
            rospy.sleep(0.1)
            wait_count += 1

        return self.navigation_status == "PAUSED"

    def resume_navigation(self):
        # FINISHED 表示导航已经结束，不需要重新发送导航目标。
        if self.navigation_status in ("FOLLOWING", "FINISHED"):
            return True
        if self.navigation_status != "PAUSED":
            rospy.logerr("导航当前状态为 %s，无法恢复", self.navigation_status)
            return False

        try:
            resume_result = self.resume_navigation_srv(TriggerRequest())
        except rospy.ServiceException as exc:
            rospy.logerr("恢复导航服务调用失败：%s", exc)
            return False
        if not resume_result.success:
            rospy.logerr("恢复导航失败：%s", resume_result.message)
            return False

        wait_count = 0
        while (not rospy.is_shutdown() and
               self.navigation_status != "FOLLOWING" and wait_count < 50):
            rospy.sleep(0.1)
            wait_count += 1
        return self.navigation_status == "FOLLOWING"

    def parse_counts(self, result_text):
        community_count = 0
        non_community_count = 0
        if not result_text:
            return community_count, non_community_count

        for item in result_text.split(','):
            item = item.strip()
            if ':' not in item:
                continue

            key, value = item.split(':', 1)
            key = key.strip().lower()
            try:
                count = int(value.strip())
            except ValueError:
                continue

            if key == 'community':
                community_count += count
            elif key == 'non-community':
                non_community_count += count

        return community_count, non_community_count

    def record_person_result(self, street, result_text):
        community_count, non_community_count = self.parse_counts(result_text)
        total_count = community_count + non_community_count
        self.total_person_count += total_count
        self.street_person_counts[street] += total_count
        self.street_non_community_counts[street] += non_community_count
        rospy.loginfo(
            "%s 街检测到 %d 人，其中社区人员 %d 人，非社区人员 %d 人。",
            street, total_count, community_count, non_community_count)

    def is_near_waypoint(self, waypoint_index):
        # 只有真正接近任务航点时才触发任务，避免被 navigation_index 提前推进影响。
        if self.current_position is None:
            return False
        if waypoint_index < 0 or waypoint_index >= len(self.waypoint_positions):
            return False

        robot_x, robot_y = self.current_position
        waypoint_x, waypoint_y = self.waypoint_positions[waypoint_index]
        distance = math.hypot(waypoint_x - robot_x, waypoint_y - robot_y)
        return distance <= self.task_waypoint_tolerance

    def check_traffic_light(self, waypoint_number):
        try:
            response = self.traffic_light_client(detectRequest(detect_flag=4))
        except rospy.ServiceException as exc:
            rospy.logerr("红绿灯检测失败：%s", exc)
            return False

        state = (response.result or "").strip().lower() or "unknown"
        rospy.loginfo("航点 %d 红绿灯识别结果：%s", waypoint_number, state)
        if state != "red":
            return True

        rospy.loginfo("航点 %d 检测到红灯，暂停导航", waypoint_number)
        if not self.pause_navigation():
            return False

        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            # 红灯等待期间持续保持零速度，避免其他速度发布者让机器人移动。
            self.cmd_vel_pub.publish(Twist())
            try:
                response = self.traffic_light_client(detectRequest(detect_flag=4))
            except rospy.ServiceException as exc:
                rospy.logwarn_throttle(5.0, "红绿灯检测失败：%s", exc)
                rate.sleep()
                continue

            state = (response.result or "").strip().lower() or "unknown"
            if state in ("green", "yellow"):
                rospy.loginfo(
                    "航点 %d 检测到 %s，恢复导航", waypoint_number, state)
                return self.resume_navigation()
            rate.sleep()
        return False

    def run_person_task(self, waypoint_index):
        """在指定航点转向识别人，并按需执行补充视觉检查。"""
        waypoint_number = waypoint_index + 1
        if not self.pause_navigation():
            return False

        try:
            target_yaw_deg = self.get_waypoint_yaw_deg(waypoint_index)
            if target_yaw_deg is None:
                self.resume_navigation()
                return False

            if waypoint_number == 6:
                # 先转 90 度识别一侧，再按旧版方式转到相反方向识别另一侧。
                first_yaw_deg = int(round(
                    self.normalize_degrees(target_yaw_deg)))
                opposite_yaw_deg = int(round(
                    self.normalize_degrees(first_yaw_deg + 180)))
                self.rotate_to_yaw(first_yaw_deg)
                first_result = self.person_client(detectRequest(detect_flag=1))
                self.record_person_result("A", first_result.result)
                self.rotate_to_yaw(opposite_yaw_deg)
                second_result = self.person_client(detectRequest(detect_flag=2))
                self.record_person_result("B", second_result.result)
            elif waypoint_number == 8:
                # 航点 8 的红绿灯位于固定方向，人员识别结束后统一恢复到
                # 1.57 rad（约 90 度），不依赖识别前可能不准确的瞬时朝向。
                traffic_light_yaw_rad = 1.57
                # rotate_to_yaw() 的参数单位是度；90 度即约 1.57 弧度。
                traffic_light_yaw_deg = 90
                if not self.wait_for_yaw():
                    self.resume_navigation()
                    return False
                self.rotate_to_yaw(target_yaw_deg)
                result = self.person_client(detectRequest(detect_flag=1))
                self.record_person_result("B", result.result)
                self.rotate_to_yaw(traffic_light_yaw_deg)
                if not self.wait_for_target_yaw(traffic_light_yaw_deg):
                    rospy.logerr(
                        "航点 %d 未恢复到固定朝向 %.2f rad，跳过红绿灯识别",
                        waypoint_number, traffic_light_yaw_rad)
                    self.resume_navigation()
                    return False
                if not self.check_traffic_light(waypoint_number):
                    self.resume_navigation()
                    return False
            else:
                self.rotate_to_yaw(target_yaw_deg)
                result = self.person_client(detectRequest(detect_flag=1))
                street = "A" if waypoint_number in (2, 4) else "B"
                self.record_person_result(street, result.result)
        except rospy.ServiceException as exc:
            rospy.logerr("行人检测服务失败：%s", exc)
            self.resume_navigation()
            return False

        return self.resume_navigation()

    def run_plate_task(self, waypoint_index):
        """按航点要求调整朝向并调用一次车牌识别服务。"""
        waypoint_number = waypoint_index + 1
        if not self.pause_navigation():
            return False

        try:
            # ID 10 的车牌朝向固定为 3.14 rad（约 180 度）。
            if waypoint_number == 10:
                self.rotate_to_yaw(180)
            else:
                target_yaw_deg = self.get_waypoint_yaw_deg(waypoint_index)
                if target_yaw_deg is None:
                    self.resume_navigation()
                    return False
                self.rotate_to_yaw(target_yaw_deg)
            result = self.plate_client(detectRequest(detect_flag=3))
        except rospy.ServiceException as exc:
            rospy.logerr("车牌检测服务失败：%s", exc)
            self.resume_navigation()
            return False

        plate_number = result.result.strip() if result.result else "识别失败"
        rospy.loginfo("车牌识别结果：%s", plate_number)
        return self.resume_navigation()

    def run(self):
        rate = rospy.Rate(10)

        try:
            start_result = self.start_navigation_srv(TriggerRequest())
        except rospy.ServiceException as exc:
            rospy.logerr("启动航点导航服务调用失败：%s", exc)
            return
        if not start_result.success:
            rospy.logerr("启动航点导航失败：%s", start_result.message)
            return

        rospy.loginfo("航点导航已启动")

        while not rospy.is_shutdown():
            if self.navigation_status == "ERROR":
                rospy.logerr("导航进入错误状态")
                return

            for waypoint_index in self.traffic_light_waypoints:
                if (waypoint_index not in self.completed_traffic_light_waypoints and
                        self.navigation_index >= waypoint_index and
                        self.is_near_waypoint(waypoint_index)):
                    if not self.check_traffic_light(waypoint_index + 1):
                        return
                    self.completed_traffic_light_waypoints.add(waypoint_index)

            for waypoint_index in self.person_waypoints:
                if (waypoint_index not in self.completed_person_waypoints and
                        self.navigation_index >= waypoint_index and
                        self.is_near_waypoint(waypoint_index)):
                    if not self.run_person_task(waypoint_index):
                        return
                    self.completed_person_waypoints.add(waypoint_index)

            for waypoint_index in self.plate_waypoints:
                if (waypoint_index not in self.completed_plate_waypoints and
                        self.navigation_index >= waypoint_index and
                        self.is_near_waypoint(waypoint_index)):
                    if not self.run_plate_task(waypoint_index):
                        return
                    self.completed_plate_waypoints.add(waypoint_index)

            # FINISHED 可能在最后一个航点的识别任务执行前到达，因此必须在
            # 完成上述任务检查后再结束任务控制器。
            if self.navigation_status == "FINISHED":
                all_tasks_done = (
                    len(self.completed_traffic_light_waypoints) ==
                    len(self.traffic_light_waypoints) and
                    len(self.completed_person_waypoints) ==
                    len(self.person_waypoints) and
                    len(self.completed_plate_waypoints) ==
                    len(self.plate_waypoints))
                if all_tasks_done:
                    rospy.loginfo("导航和视觉任务已完成")
                else:
                    rospy.logwarn("导航结束，仍有视觉任务未完成")
                rospy.loginfo(
                    "共检测到 %d 人；其中 A 街总人数 %d 人，B 街总人数 %d 人；"
                    "发现 A 街非社区人员 %d 人，B 街非社区人员 %d 人",
                    self.total_person_count,
                    self.street_person_counts["A"],
                    self.street_person_counts["B"],
                    self.street_non_community_counts["A"],
                    self.street_non_community_counts["B"])
                return

            rate.sleep()

    def shutdown(self):
        try:
            if self.navigation_status in ["FOLLOWING", "PAUSED"]:
                self.stop_navigation_srv(TriggerRequest())
        except rospy.ServiceException:
            pass

        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(0.5)


if __name__ == '__main__':
    try:
        MissionController().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("任务流程结束")
