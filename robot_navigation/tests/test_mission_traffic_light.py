import importlib.util
import math
import unittest
from pathlib import Path
from unittest.mock import Mock, call, patch


PACKAGE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("mission", PACKAGE / "scripts/mission.py")
mission = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mission)


class MissionTrafficLightTest(unittest.TestCase):
    def setUp(self):
        self.node = mission.MissionController.__new__(mission.MissionController)
        self.node.traffic_light_client = Mock()
        self.node.pause_navigation = Mock(return_value=True)
        self.node.resume_navigation = Mock(return_value=True)
        self.node.cmd_vel_pub = Mock()
        self.node.total_person_count = 0
        self.node.street_person_counts = {"A": 0, "B": 0}
        self.node.street_non_community_counts = {"A": 0, "B": 0}

    def test_odom_pose_is_transformed_to_waypoint_frame(self):
        self.node.waypoint_frame = "map"
        self.node.tf_listener = Mock()
        odom = mission.Odometry()
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = 8.0
        odom.pose.pose.orientation.w = 1.0
        transformed = mission.PoseStamped()
        transformed.header.frame_id = "map"
        transformed.pose.position.x = 0.05
        transformed.pose.orientation.z = math.sin(math.pi / 4)
        transformed.pose.orientation.w = math.cos(math.pi / 4)
        self.node.tf_listener.transformPose.return_value = transformed
        self.node.waypoint_positions = [(0, 0)] * 12
        self.node.task_waypoint_tolerance = 0.1

        self.node.odom_callback(odom)

        self.node.tf_listener.transformPose.assert_called_once()
        self.assertEqual(self.node.tf_listener.transformPose.call_args.args[0], "map")
        self.assertEqual(self.node.current_position, (0.05, 0))
        self.assertAlmostEqual(self.node.current_yaw, math.pi / 2)
        self.assertTrue(self.node.is_near_waypoint(9))

    def test_odom_in_waypoint_frame_needs_no_transform(self):
        self.node.waypoint_frame = "map"
        self.node.tf_listener = Mock()
        odom = mission.Odometry()
        odom.header.frame_id = "map"
        odom.pose.pose.position.x = 2.0
        odom.pose.pose.orientation.w = 1.0
        self.node.odom_callback(odom)
        self.assertEqual(self.node.current_position, (2.0, 0))
        self.node.tf_listener.transformPose.assert_not_called()

    def test_waypoint_10_uses_map_position_to_trigger_plate_request(self):
        for map_distance, expected_calls in ((0.05, 1), (0.15, 0)):
            with self.subTest(map_distance=map_distance):
                self.configure_run_node(9, set())
                self.node.completed_traffic_light_waypoints = {0, 8}
                self.node.completed_person_waypoints = {1, 3, 5, 7}
                self.node.completed_plate_waypoints = {10, 11}
                self.node.waypoint_frame = "map"
                self.node.tf_listener = Mock()
                self.node.task_waypoint_tolerance = 0.1
                self.node.waypoint_positions = [(0, 0)] * 12
                self.node.is_near_waypoint = (
                    mission.MissionController.is_near_waypoint.__get__(self.node))
                odom = mission.Odometry()
                odom.header.frame_id = "odom"
                odom.pose.pose.position.x = 8.0
                transformed = mission.PoseStamped()
                transformed.pose.position.x = map_distance
                transformed.pose.orientation.w = 1.0
                self.node.tf_listener.transformPose.return_value = transformed
                self.node.rotate_to_yaw = Mock()
                self.node.plate_client = Mock(return_value=Mock(result="沪A12345"))
                self.node.odom_callback(odom)
                with patch.object(mission.rospy, "is_shutdown", side_effect=[False, True]), \
                     patch.object(mission.rospy, "Rate"), \
                     patch.object(mission.rospy, "loginfo"):
                    self.node.run()
                self.assertEqual(self.node.plate_client.call_count, expected_calls)
                if expected_calls:
                    self.node.rotate_to_yaw.assert_called_once_with(180)
                else:
                    self.node.rotate_to_yaw.assert_not_called()
                if expected_calls:
                    self.node.plate_client.assert_called_once_with(
                        mission.detectRequest(detect_flag=3))

    def test_missing_transform_does_not_trigger_from_stale_position(self):
        self.node.waypoint_frame = "map"
        self.node.tf_listener = Mock()
        self.node.current_position = (0, 0)
        self.node.current_yaw = 0
        self.node.tf_listener.transformPose.side_effect = mission.tf.LookupException("missing")
        odom = mission.Odometry()
        odom.header.frame_id = "odom"
        with patch.object(mission.rospy, "logwarn_throttle"):
            self.node.odom_callback(odom)
        self.assertIsNone(self.node.current_position)
        self.assertFalse(self.node.is_near_waypoint(9))

    def test_odom_without_frame_is_rejected(self):
        self.node.current_position = (0, 0)
        with patch.object(mission.rospy, "logwarn_throttle"):
            self.node.odom_callback(mission.Odometry())
        self.assertIsNone(self.node.current_position)

    def configure_run_node(self, navigation_index, near_indices):
        self.node.start_navigation_srv = Mock(return_value=Mock(success=True))
        self.node.navigation_status = "FOLLOWING"
        self.node.navigation_index = navigation_index
        self.node.traffic_light_waypoints = [0, 8]
        self.node.person_waypoints = [1, 3, 5, 7]
        self.node.plate_waypoints = [9, 10, 11]
        self.node.completed_traffic_light_waypoints = set()
        self.node.completed_person_waypoints = set()
        self.node.completed_plate_waypoints = set()
        self.node.is_near_waypoint = Mock(
            side_effect=lambda index: index in near_indices)

    def test_ui_waypoint_ids_are_converted_to_zero_based_indices(self):
        with patch.object(mission.rospy, "get_param", return_value=[2, 4, 2, 0, "bad", -1]):
            self.assertEqual(
                self.node._get_waypoint_ids("~person_waypoints", []), [1, 3])

    def test_non_red_states_continue_without_pause(self):
        for state in ("green", "yellow", "unknown"):
            with self.subTest(state=state):
                self.node.traffic_light_client.return_value.result = state
                self.assertTrue(self.node.check_traffic_light(1))
        self.node.pause_navigation.assert_not_called()

    def test_traffic_light_result_is_logged_for_non_red_states(self):
        for state in ("green", "yellow", "unknown"):
            with self.subTest(state=state):
                self.node.traffic_light_client.return_value.result = state
                with patch.object(mission.rospy, "loginfo") as loginfo:
                    self.assertTrue(self.node.check_traffic_light(3))
                self.assertIn(
                    call("航点 %d 红绿灯识别结果：%s", 3, state),
                    loginfo.call_args_list)

    def test_red_waits_through_unknown_until_green_or_yellow(self):
        for final_state in ("green", "yellow"):
            with self.subTest(final_state=final_state):
                self.node.pause_navigation.reset_mock()
                self.node.resume_navigation.reset_mock()
                self.node.traffic_light_client.side_effect = [
                    Mock(result="red"), Mock(result="unknown"), Mock(result=final_state)]
                with patch.object(mission.rospy, "is_shutdown", return_value=False), \
                     patch.object(mission.rospy, "Rate") as rate, \
                     patch.object(mission.rospy, "loginfo"):
                    self.assertTrue(self.node.check_traffic_light(6))
                self.node.pause_navigation.assert_called_once_with()
                self.node.resume_navigation.assert_called_once_with()
                self.assertEqual(rate.return_value.sleep.call_count, 1)
                self.assertEqual(self.node.cmd_vel_pub.publish.call_args.args[0].linear.x, 0)

    def test_red_polling_results_are_logged(self):
        self.node.traffic_light_client.side_effect = [
            Mock(result="red"), Mock(result="unknown"), Mock(result="green")]
        with patch.object(mission.rospy, "is_shutdown", return_value=False), \
                patch.object(mission.rospy, "Rate"), \
                patch.object(mission.rospy, "loginfo") as loginfo:
            self.assertTrue(self.node.check_traffic_light(6))
        self.assertIn(
            call("航点 %d 红绿灯识别结果：%s", 6, "red"),
            loginfo.call_args_list)
        self.assertIn(
            call("航点 %d 红灯等待期间识别结果：%s", 6, "unknown"),
            loginfo.call_args_list)
        self.assertIn(
            call("航点 %d 红灯等待期间识别结果：%s", 6, "green"),
            loginfo.call_args_list)

    def test_service_failure_does_not_pass_red_light(self):
        self.node.traffic_light_client.side_effect = [
            Mock(result="red"), mission.rospy.ServiceException("offline"),
            Mock(result="yellow")]
        with patch.object(mission.rospy, "is_shutdown", return_value=False), \
             patch.object(mission.rospy, "Rate") as rate, \
             patch.object(mission.rospy, "loginfo"), \
             patch.object(mission.rospy, "logwarn_throttle"):
            self.assertTrue(self.node.check_traffic_light(1))
        self.node.resume_navigation.assert_called_once_with()
        rate.return_value.sleep.assert_called_once_with()

    def test_run_checks_configured_traffic_light_waypoints_once(self):
        self.configure_run_node(8, {0, 8})
        self.node.check_traffic_light = Mock(return_value=True)
        with patch.object(mission.rospy, "is_shutdown", side_effect=[False, True]), \
             patch.object(mission.rospy, "Rate"), \
             patch.object(mission.rospy, "loginfo"):
            self.node.run()
        self.assertEqual(self.node.check_traffic_light.call_args_list, [call(1), call(9)])
        self.assertEqual(self.node.completed_traffic_light_waypoints, {0, 8})

    def test_person_waypoint_rotates_and_calls_person_service(self):
        self.configure_run_node(3, {3})
        self.node.completed_traffic_light_waypoints = {0, 8}
        self.node.completed_person_waypoints = {1, 5, 7}
        self.node.completed_plate_waypoints = {9, 10, 11}
        self.node.pause_navigation = Mock(return_value=True)
        self.node.resume_navigation = Mock(return_value=True)
        self.node.get_waypoint_yaw_deg = Mock(return_value=90)
        self.node.rotate_to_yaw = Mock()
        self.node.person_client = Mock(
            return_value=Mock(result="community:2, non-community:1"))
        with patch.object(mission.rospy, "is_shutdown", side_effect=[False, True]), \
             patch.object(mission.rospy, "Rate"), \
             patch.object(mission.rospy, "loginfo"):
            self.node.run()

        self.node.person_client.assert_called_once()
        self.assertEqual(
            self.node.person_client.call_args.args[0].detect_flag, 1)
        self.node.rotate_to_yaw.assert_called_once_with(90)
        self.assertEqual(self.node.completed_person_waypoints, {1, 3, 5, 7})

    def test_person_waypoint_6_checks_both_sides(self):
        # 与 mission_old.py 一致：直接使用 target，再使用 target + 180。
        self.node.pause_navigation = Mock(return_value=True)
        self.node.resume_navigation = Mock(return_value=True)
        self.node.get_waypoint_yaw_deg = Mock(return_value=90)
        self.node.rotate_to_yaw = Mock()
        self.node.person_client = Mock(
            side_effect=[Mock(result="community:1"), Mock(result="non-community:2")])
        actions = Mock()
        actions.attach_mock(self.node.rotate_to_yaw, "rotate")
        actions.attach_mock(self.node.person_client, "person")

        with patch.object(mission.rospy, "loginfo"):
            self.assertTrue(self.node.run_person_task(5))

        self.assertEqual(self.node.person_client.call_count, 2)
        self.assertEqual(
            self.node.rotate_to_yaw.call_args_list, [call(90), call(-90)])
        self.assertEqual(
            [item.args[0].detect_flag for item in self.node.person_client.call_args_list],
            [1, 2])
        self.assertEqual(actions.mock_calls, [
            call.rotate(90), call.person(mission.detectRequest(detect_flag=1)),
            call.rotate(-90), call.person(mission.detectRequest(detect_flag=2))])
        self.assertEqual(self.node.total_person_count, 3)
        self.assertEqual(self.node.street_person_counts, {"A": 1, "B": 2})
        self.assertEqual(self.node.street_non_community_counts, {"A": 0, "B": 2})

    def test_person_waypoint_6_uses_target_without_extra_offset(self):
        for target, opposite in ((0, 180), (-90, 90), (180, 0)):
            with self.subTest(target=target):
                self.node.get_waypoint_yaw_deg = Mock(return_value=target)
                self.node.rotate_to_yaw = Mock()
                self.node.person_client = Mock(return_value=Mock(result="community:1"))
                with patch.object(mission.rospy, "loginfo"):
                    self.assertTrue(self.node.run_person_task(5))
                self.assertEqual(self.node.rotate_to_yaw.call_args_list,
                                 [call(target), call(opposite)])

    def test_person_waypoint_8_checks_traffic_light_after_turning(self):
        self.node.pause_navigation = Mock(return_value=True)
        self.node.resume_navigation = Mock(return_value=True)
        self.node.get_waypoint_yaw_deg = Mock(return_value=0)
        self.node.current_yaw = math.pi / 2
        self.node.wait_for_yaw = Mock(return_value=True)
        self.node.rotate_to_yaw = Mock(
            side_effect=lambda yaw: setattr(self.node, "current_yaw", math.radians(yaw)))
        self.node.person_client = Mock(return_value=Mock(result="community:1"))
        self.node.check_traffic_light = Mock(return_value=True)
        actions = Mock()
        actions.attach_mock(self.node.rotate_to_yaw, "rotate")
        actions.attach_mock(self.node.person_client, "person")
        actions.attach_mock(self.node.check_traffic_light, "traffic_light")

        with patch.object(mission.rospy, "loginfo"):
            self.assertTrue(self.node.run_person_task(7))

        self.node.check_traffic_light.assert_called_once_with(8)
        self.assertEqual(
            self.node.rotate_to_yaw.call_args_list, [call(0), call(90)])
        self.assertEqual(actions.mock_calls, [
            call.rotate(0), call.person(mission.detectRequest(detect_flag=1)),
            call.rotate(90), call.traffic_light(8)])
        self.assertEqual(self.node.street_person_counts, {"A": 0, "B": 1})

    def test_person_waypoint_8_restores_fixed_traffic_light_heading(self):
        self.node.pause_navigation = Mock(return_value=True)
        self.node.resume_navigation = Mock(return_value=True)
        self.node.get_waypoint_yaw_deg = Mock(return_value=0)
        self.node.current_yaw = math.radians(180)
        self.node.wait_for_yaw = Mock(return_value=True)
        self.node.rotate_to_yaw = Mock()
        self.node.person_client = Mock(return_value=Mock(result="community:1"))
        self.node.wait_for_target_yaw = Mock(return_value=True)
        self.node.check_traffic_light = Mock(return_value=True)

        with patch.object(mission.rospy, "loginfo"):
            self.assertTrue(self.node.run_person_task(7))

        self.assertEqual(
            self.node.rotate_to_yaw.call_args_list, [call(0), call(90)])
        self.node.wait_for_target_yaw.assert_called_once_with(90)
        self.node.check_traffic_light.assert_called_once_with(8)

    def test_person_results_accumulate_by_street(self):
        self.node.get_waypoint_yaw_deg = Mock(return_value=90)
        self.node.rotate_to_yaw = Mock()
        self.node.current_yaw = 0
        self.node.wait_for_yaw = Mock(return_value=True)
        self.node.wait_for_target_yaw = Mock(return_value=True)
        self.node.check_traffic_light = Mock(return_value=True)
        self.node.person_client = Mock(side_effect=[
            Mock(result="community:2, non-community:1"),
            Mock(result="community:3, non-community:2"),
            Mock(result="community:4, non-community:3"),
            Mock(result="community:5, non-community:4"),
            Mock(result="community:6, non-community:5"),
            Mock(result="community:7, non-community:6")])

        with patch.object(mission.rospy, "loginfo") as loginfo:
            for waypoint_index in (1, 3, 5, 7, 8):
                self.assertTrue(self.node.run_person_task(waypoint_index))

        self.assertEqual(self.node.total_person_count, 48)
        self.assertEqual(self.node.street_person_counts, {"A": 15, "B": 33})
        self.assertEqual(self.node.street_non_community_counts, {"A": 6, "B": 15})
        self.assertEqual([item.args[1] for item in loginfo.call_args_list],
                         ["A", "A", "A", "B", "B", "B"])

    def test_empty_or_malformed_person_results_do_not_add_people(self):
        with patch.object(mission.rospy, "loginfo"):
            for result_text in (None, "", "unknown", "community:bad, non-community:"):
                with self.subTest(result_text=result_text):
                    self.node.record_person_result("A", result_text)

        self.assertEqual(self.node.total_person_count, 0)
        self.assertEqual(self.node.street_person_counts, {"A": 0, "B": 0})
        self.assertEqual(self.node.street_non_community_counts, {"A": 0, "B": 0})

    def test_person_service_failure_preserves_only_successful_results(self):
        self.node.get_waypoint_yaw_deg = Mock(return_value=90)
        self.node.rotate_to_yaw = Mock()
        self.node.person_client = Mock(side_effect=[
            Mock(result="community:2, non-community:1"),
            mission.rospy.ServiceException("offline")])
        with patch.object(mission.rospy, "loginfo"), \
             patch.object(mission.rospy, "logerr"):
            self.assertFalse(self.node.run_person_task(5))

        self.assertEqual(self.node.total_person_count, 3)
        self.assertEqual(self.node.street_person_counts, {"A": 3, "B": 0})
        self.assertEqual(self.node.street_non_community_counts, {"A": 1, "B": 0})

    def test_finished_mission_logs_accumulated_person_summary_once(self):
        self.configure_run_node(7, {7})
        self.node.navigation_status = "FINISHED"
        self.node.completed_traffic_light_waypoints = {0, 8}
        self.node.completed_person_waypoints = {1, 3, 5}
        self.node.completed_plate_waypoints = {9, 10, 11}
        self.node.total_person_count = 5
        self.node.street_person_counts = {"A": 3, "B": 2}
        self.node.street_non_community_counts = {"A": 1, "B": 1}
        self.node.get_waypoint_yaw_deg = Mock(return_value=90)
        self.node.rotate_to_yaw = Mock()
        self.node.current_yaw = 0
        self.node.wait_for_yaw = Mock(return_value=True)
        self.node.wait_for_target_yaw = Mock(return_value=True)
        self.node.check_traffic_light = Mock(return_value=True)
        self.node.person_client = Mock(return_value=Mock(result="community:2, non-community:3"))
        with patch.object(mission.rospy, "is_shutdown", return_value=False), \
             patch.object(mission.rospy, "Rate"), \
             patch.object(mission.rospy, "loginfo") as loginfo:
            self.node.run()

        summary = call(
            "共检测到 %d 人；其中 A 街总人数 %d 人，B 街总人数 %d 人；"
            "发现 A 街非社区人员 %d 人，B 街非社区人员 %d 人",
            10, 3, 7, 1, 4)
        self.assertEqual(loginfo.call_args_list.count(summary), 1)
        self.assertEqual(loginfo.call_args, summary)
        self.assertTrue(all("航点" not in item.args[0]
                            for item in loginfo.call_args_list))

    def test_person_waypoint_8_requires_yaw_before_turning(self):
        self.node.get_waypoint_yaw_deg = Mock(return_value=0)
        self.node.wait_for_yaw = Mock(return_value=False)
        self.node.rotate_to_yaw = Mock()
        self.node.person_client = Mock()
        self.node.check_traffic_light = Mock()
        with patch.object(mission.rospy, "loginfo"):
            self.assertFalse(self.node.run_person_task(7))
        self.node.rotate_to_yaw.assert_not_called()
        self.node.person_client.assert_not_called()
        self.node.check_traffic_light.assert_not_called()
        self.node.resume_navigation.assert_called_once()

    def test_person_waypoint_8_skips_traffic_light_when_lane_yaw_not_restored(self):
        self.node.get_waypoint_yaw_deg = Mock(return_value=0)
        self.node.current_yaw = math.pi / 2
        self.node.wait_for_yaw = Mock(return_value=True)
        self.node.rotate_to_yaw = Mock(
            side_effect=lambda yaw: setattr(
                self.node, "current_yaw", math.radians(yaw)))
        self.node.person_client = Mock(return_value=Mock(result="community:1"))
        self.node.wait_for_target_yaw = Mock(return_value=False)
        self.node.check_traffic_light = Mock()
        with patch.object(mission.rospy, "loginfo"), \
                patch.object(mission.rospy, "logerr"):
            self.assertFalse(self.node.run_person_task(7))

        self.node.rotate_to_yaw.assert_has_calls([call(0), call(90)])
        self.node.person_client.assert_called_once()
        self.node.wait_for_target_yaw.assert_called_once_with(90.0)
        self.node.check_traffic_light.assert_not_called()
        self.node.resume_navigation.assert_called_once()

    def test_wait_for_target_yaw_rejects_wrong_heading(self):
        self.node.current_yaw = 0.0
        self.node.wait_for_yaw = Mock(return_value=True)
        self.node.turn_tolerance_deg = 2.0
        self.node.turn_verify_timeout = 0.0
        with patch.object(mission.rospy, "is_shutdown", return_value=True), \
                patch.object(mission.rospy, "Rate"), \
                patch.object(mission.rospy, "logwarn") as logwarn:
            self.assertFalse(self.node.wait_for_target_yaw(90))
        logwarn.assert_called_once()

    def test_waypoint_10_plate_recognition_preserves_distance_gate(self):
        for distance, expected_calls in ((0.05, 1), (0.1, 1), (0.15, 0)):
            with self.subTest(distance=distance):
                self.configure_run_node(9, set())
                self.node.completed_traffic_light_waypoints = {0, 8}
                self.node.completed_person_waypoints = {1, 3, 5, 7}
                self.node.completed_plate_waypoints = {10, 11}
                self.node.current_position = (distance, 0)
                self.node.waypoint_positions = [(0, 0)] * 12
                self.node.task_waypoint_tolerance = 0.1
                self.node.is_near_waypoint = (
                    mission.MissionController.is_near_waypoint.__get__(self.node))
                self.node.get_waypoint_yaw_deg = Mock(return_value=180)
                self.node.rotate_to_yaw = Mock()
                self.node.plate_client = Mock(return_value=Mock(result="ABC123"))
                with patch.object(mission.rospy, "is_shutdown", side_effect=[False, True]), \
                     patch.object(mission.rospy, "Rate"), \
                     patch.object(mission.rospy, "loginfo"):
                    self.node.run()

                self.assertEqual(self.node.plate_client.call_count, expected_calls)
                if expected_calls:
                    self.node.rotate_to_yaw.assert_called_once_with(180)
                else:
                    self.node.rotate_to_yaw.assert_not_called()
                self.node.get_waypoint_yaw_deg.assert_not_called()
                if expected_calls:
                    self.node.plate_client.assert_called_once_with(
                        mission.detectRequest(detect_flag=3))
                    self.assertIn(9, self.node.completed_plate_waypoints)
                else:
                    self.node.rotate_to_yaw.assert_not_called()
                    self.assertNotIn(9, self.node.completed_plate_waypoints)

    def test_plate_waypoint_10_rotates_to_fixed_heading_before_plate_service(self):
        self.configure_run_node(9, {9})
        self.node.completed_traffic_light_waypoints = {0, 8}
        self.node.completed_person_waypoints = {1, 3, 5, 7}
        self.node.completed_plate_waypoints = {10, 11}
        self.node.pause_navigation = Mock(return_value=True)
        self.node.resume_navigation = Mock(return_value=True)
        self.node.get_waypoint_yaw_deg = Mock(return_value=0)
        self.node.rotate_to_yaw = Mock()
        self.node.plate_client = Mock(return_value=Mock(result="ABC123"))
        actions = Mock()
        actions.attach_mock(self.node.rotate_to_yaw, "rotate")
        actions.attach_mock(self.node.plate_client, "plate")
        with patch.object(mission.rospy, "is_shutdown", side_effect=[False, True]), \
             patch.object(mission.rospy, "Rate"), \
            patch.object(mission.rospy, "loginfo"):
            self.node.run()

        self.assertEqual(self.node.plate_client.call_args.args[0].detect_flag, 3)
        self.node.rotate_to_yaw.assert_called_once_with(180)
        self.node.get_waypoint_yaw_deg.assert_not_called()
        self.assertEqual(actions.mock_calls, [
            call.rotate(180),
            call.plate(mission.detectRequest(detect_flag=3))])
        self.assertEqual(self.node.completed_plate_waypoints, {9, 10, 11})

    def test_plate_waypoints_11_and_12_rotate_to_target_before_recognition(self):
        for waypoint_index in (10, 11):
            with self.subTest(waypoint_index=waypoint_index):
                self.node.get_waypoint_yaw_deg = Mock(return_value=180)
                self.node.rotate_to_yaw = Mock()
                self.node.plate_client = Mock(return_value=Mock(result="XYZ789"))
                actions = Mock()
                actions.attach_mock(self.node.rotate_to_yaw, "rotate")
                actions.attach_mock(self.node.plate_client, "plate")
                with patch.object(mission.rospy, "loginfo"):
                    self.assertTrue(self.node.run_plate_task(waypoint_index))
                self.assertEqual(actions.mock_calls, [
                    call.rotate(180), call.plate(mission.detectRequest(detect_flag=3))])

    def test_last_configured_task_is_processed_before_finished_status_exits(self):
        self.configure_run_node(11, {11})
        self.node.navigation_status = "FINISHED"
        self.node.completed_traffic_light_waypoints = {0, 8}
        self.node.completed_person_waypoints = {1, 3, 5, 7}
        self.node.completed_plate_waypoints = {9, 10}
        self.node.pause_navigation = Mock(return_value=True)
        self.node.resume_navigation = Mock(return_value=True)
        self.node.get_waypoint_yaw_deg = Mock(return_value=0)
        self.node.rotate_to_yaw = Mock()
        self.node.plate_client = Mock(return_value=Mock(result="ABC123"))
        with patch.object(mission.rospy, "is_shutdown", return_value=False), \
             patch.object(mission.rospy, "Rate"), \
             patch.object(mission.rospy, "loginfo"), \
             patch.object(mission.rospy, "logwarn"):
            self.node.run()

        self.node.plate_client.assert_called_once()
        self.assertEqual(self.node.completed_plate_waypoints, {9, 10, 11})


if __name__ == "__main__":
    unittest.main()
