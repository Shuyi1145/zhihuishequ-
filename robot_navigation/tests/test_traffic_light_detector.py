import importlib.util
import threading
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import cv2
import numpy as np
from cv_bridge import CvBridgeError


PACKAGE = Path(__file__).resolve().parents[1]
TEXTURES = PACKAGE.parent / "robot_simulation/models/traffic_light/materials/textures"
spec = importlib.util.spec_from_file_location(
    "traffic_light_detector", PACKAGE / "scripts/traffic_light_detector.py")
detector = importlib.util.module_from_spec(spec)
spec.loader.exec_module(detector)


def camera_frame(active=None, lamp_size=48):
    """用模型原图生成一张含箱体和三盏灯的相机帧。"""
    frame = np.full((480, 640, 3), 100, dtype=np.uint8)
    gap = round(lamp_size * 0.2 / 0.13)
    centers = (320 - gap, 320, 320 + gap)
    top = 240 - lamp_size // 2
    cv2.rectangle(frame, (centers[0] - lamp_size // 2 - 5, top - 5),
                  (centers[2] + lamp_size // 2 + 5, top + lamp_size + 5),
                  (15, 15, 15), -1)
    for color, center in zip(("red", "yellow", "green"), centers):
        state = "on" if color == active or isinstance(active, tuple) and color in active else "dark"
        texture = cv2.imread(str(TEXTURES / (color + "_" + state + ".png")))
        assert texture is not None
        x = center - lamp_size // 2
        frame[top:top + lamp_size, x:x + lamp_size] = cv2.resize(
            texture, (lamp_size, lamp_size), interpolation=cv2.INTER_AREA)
    return frame


class DetectLightTest(unittest.TestCase):
    def setUp(self):
        self.output_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.output_dir.cleanup)

    def test_each_real_texture_is_classified_near_and_far(self):
        for size in (48, 12):
            for color in ("red", "yellow", "green"):
                with self.subTest(size=size, color=color):
                    state, box = detector.detect_light(camera_frame(color, size))
                    self.assertEqual(state, color)
                    self.assertIsNotNone(box)
                    self.assertGreater(box[2], 0)

    def test_no_light_or_ambiguous_lights_return_unknown(self):
        self.assertEqual(detector.detect_light(camera_frame())[0], "unknown")
        self.assertEqual(detector.detect_light(camera_frame(("red", "green")))[0],
                         "unknown")

    def test_roi_and_shape_filter_reject_unrelated_regions(self):
        frame = camera_frame("red")
        self.assertEqual(detector.detect_light(frame, roi=(0.45, 0.3, 0.8, 0.7))[0],
                         "unknown")
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.rectangle(frame, (100, 100), (220, 104), (0, 0, 255), -1)
        self.assertEqual(detector.detect_light(frame)[0], "unknown")

    def test_invalid_inputs_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "BGR"):
            detector.detect_light(None)
        with self.assertRaisesRegex(ValueError, "roi"):
            detector.detect_light(camera_frame("red"), roi=(0.8, 0, 0.2, 1))
        with self.assertRaisesRegex(ValueError, "roi"):
            detector.roi_bounds(640, 480, None)
        with self.assertRaisesRegex(ValueError, "threshold"):
            detector.detect_light(camera_frame("red"), min_pixels=0)

    def test_callback_publishes_state_and_unknown_on_conversion_error(self):
        node = detector.TrafficLightDetector.__new__(detector.TrafficLightDetector)
        node.save_dir = self.output_dir.name
        node.result_image_index = 0
        node._red_sequence_saved = False
        node.bridge = Mock()
        node.bridge.imgmsg_to_cv2.return_value = camera_frame("green")
        node.image_lock = threading.Lock()
        node.latest_image = None
        node.state_pub = Mock()
        node.debug_pub = None
        node.roi = (0, 0, 1, 1)
        node.min_pixels = 12
        node.min_saturation = 110
        node.min_value = 200
        node.image_callback(Mock())
        self.assertEqual(node.state_pub.publish.call_args.args[0].data, "green")
        self.assertEqual(node.handle_recognition(Mock(detect_flag=4)).result, "green")

        node.bridge.imgmsg_to_cv2.side_effect = CvBridgeError("bad image")
        with patch.object(detector.rospy, "logwarn_throttle"):
            node.image_callback(Mock())
        self.assertEqual(node.state_pub.publish.call_args.args[0].data, "unknown")
        self.assertEqual(node.handle_recognition(Mock(detect_flag=4)).result, "unknown")

    def test_service_rejects_other_flags_and_reports_current_frame(self):
        node = detector.TrafficLightDetector.__new__(detector.TrafficLightDetector)
        node.save_dir = self.output_dir.name
        node.result_image_index = 0
        node._red_sequence_saved = False
        node.image_lock = threading.Lock()
        node.latest_image = None
        node.roi = (0, 0, 1, 1)
        node.min_pixels = 12
        node.min_saturation = 110
        node.min_value = 200
        self.assertEqual(node.handle_recognition(Mock(detect_flag=3)).result,
                         "UNSUPPORTED_FLAG")
        self.assertEqual(node.handle_recognition(Mock(detect_flag=4)).result, "unknown")
        for _ in range(3):
            color = "red"
            node.latest_image = camera_frame(color)
            self.assertEqual(node.handle_recognition(Mock(detect_flag=4)).result, color)
        files = list(Path(node.save_dir).glob("traffic_light_*.jpg"))
        self.assertEqual(sorted(path.name for path in files), ["traffic_light_1.jpg"])
        for filename in files:
            self.assertEqual(cv2.imread(str(filename)).shape, (480, 640, 3))

        # 红灯结束的绿灯响应不再追加图片，下一次独立识别可以重新保存。
        node.latest_image = camera_frame("green")
        node.handle_recognition(Mock(detect_flag=4))
        self.assertEqual(len(list(Path(node.save_dir).glob("traffic_light_*.jpg"))), 1)
        node.latest_image = camera_frame("green")
        node.handle_recognition(Mock(detect_flag=4))
        self.assertEqual(len(list(Path(node.save_dir).glob("traffic_light_*.jpg"))), 2)
        self.assertEqual(detector.detect_light(
            cv2.imread(str(Path(node.save_dir) / "traffic_light_1.jpg")))[0], "red")

    def test_save_failure_preserves_traffic_light_result_and_is_logged(self):
        node = detector.TrafficLightDetector.__new__(detector.TrafficLightDetector)
        node.image_lock = threading.Lock()
        node.latest_image = camera_frame("red")
        node.roi = (0, 0, 1, 1)
        node.min_pixels = 12
        node.min_saturation = 110
        node.min_value = 200
        node.save_dir = self.output_dir.name
        node.result_image_index = 0
        node._red_sequence_saved = False
        with patch.object(detector.cv2, "imwrite", return_value=False), \
             patch.object(detector.rospy, "logerr") as logerr:
            self.assertEqual(node.handle_recognition(Mock(detect_flag=4)).result, "red")
        logerr.assert_called_once()

    def test_ros_topics_and_debug_failure_preserve_detected_state(self):
        with patch.object(detector.rospy, "get_param",
                          side_effect=lambda _, default: default):
            with patch.object(detector.rospy, "Publisher") as publisher:
                with patch.object(detector.rospy, "Subscriber") as subscriber:
                    with patch.object(detector.rospy, "Service") as service:
                        with patch.object(detector.rospy, "loginfo"):
                            node = detector.TrafficLightDetector()
        publisher.assert_called_once_with("/traffic_light/state", detector.String,
                                          queue_size=1)
        self.assertEqual(subscriber.call_args.args[:2], ("/image_raw", detector.Image))
        self.assertEqual(subscriber.call_args.kwargs["queue_size"], 1)
        service.assert_called_once_with("/recognize_traffic_light", detector.detect,
                                        node.handle_recognition)

        node.bridge = Mock()
        node.bridge.imgmsg_to_cv2.return_value = camera_frame("yellow")
        node.bridge.cv2_to_imgmsg.side_effect = CvBridgeError("debug failed")
        node.debug_pub = Mock()
        node.debug_pub.get_num_connections.return_value = 1
        with patch.object(detector.rospy, "logwarn_throttle"):
            node.image_callback(Mock())
        self.assertEqual(node.state_pub.publish.call_count, 1)
        self.assertEqual(node.state_pub.publish.call_args.args[0].data, "yellow")


if __name__ == "__main__":
    unittest.main()
