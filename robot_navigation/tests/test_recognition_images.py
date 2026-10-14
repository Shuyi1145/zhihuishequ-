import importlib.util
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import cv2
import numpy as np
from std_msgs.msg import Header


PACKAGE = Path(__file__).resolve().parents[1]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


plate = load_module("detect_plate", PACKAGE / "scripts/detect_plate.py")
# 服务测试不加载权重或 PyTorch；推理输出由测试提供。
with patch.dict(sys.modules, {"torch": Mock(), "ultralytics": Mock(),
                              "yolo_ros.msg": Mock()}):
    person = load_module("detect_ros", PACKAGE.parent / "yolo_ros/src/detect_ros.py")


class RecognitionImagesTest(unittest.TestCase):
    def setUp(self):
        self.output_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.output_dir.cleanup)
        self.save_dir = str(Path(self.output_dir.name) / "results")
        self.frame = np.zeros((60, 160, 3), dtype=np.uint8)
        self.annotated = np.full_like(self.frame, 180)

    def plate_node(self):
        node = plate.PlateRecognizer.__new__(plate.PlateRecognizer)
        node.image_lock = threading.Lock()
        node.result_image_index = 0
        node.latest_image = self.frame.copy()
        node.save_dir = self.save_dir
        node.recognize_plate = Mock(return_value={"words_result": {"number": "ABC123"}})
        node.draw_plate_info = Mock(return_value=(self.annotated.copy(), "ABC123"))
        return node

    def test_chinese_plate_is_drawn_with_available_font(self):
        node = plate.PlateRecognizer.__new__(plate.PlateRecognizer)
        with patch.object(plate.rospy, "get_param", return_value=plate.FONT_PATH):
            node.font = node.load_font()
        self.assertIsNotNone(node.font)
        result = {"words_result": {
            "number": "沪A12345",
            "vertexes_location": [{"x": 105, "y": 40}, {"x": 150, "y": 40},
                                  {"x": 150, "y": 55}, {"x": 105, "y": 55}]}}
        with patch.object(plate.rospy, "loginfo"):
            image, number = node.draw_plate_info(self.frame, result)
        self.assertEqual(number, "沪A12345")
        self.assertGreater(np.count_nonzero(image[10:30, 10:100]), 0)

    def test_missing_configured_font_falls_back_to_system_chinese_font(self):
        node = plate.PlateRecognizer.__new__(plate.PlateRecognizer)
        with patch.object(plate.rospy, "get_param", return_value="/missing/font.ttc"):
            self.assertIsNotNone(node.load_font())

    def test_no_usable_font_preserves_plate_result_and_bounding_box(self):
        node = plate.PlateRecognizer.__new__(plate.PlateRecognizer)
        with patch.object(plate.rospy, "get_param", return_value=plate.FONT_PATH), \
             patch.object(plate.ImageFont, "truetype", side_effect=OSError("missing")), \
             patch.object(plate.rospy, "logerr") as logerr:
            node.font = node.load_font()
        self.assertIsNone(node.font)
        logerr.assert_called_once()
        result = {"words_result": {
            "number": "沪A12345",
            "vertexes_location": [{"x": 5, "y": 5}, {"x": 150, "y": 5},
                                  {"x": 150, "y": 15}, {"x": 5, "y": 15}]}}
        with patch.object(plate.rospy, "loginfo"):
            image, number = node.draw_plate_info(self.frame, result)
        self.assertEqual(number, "沪A12345")
        self.assertGreater(np.count_nonzero(image), 0)

    def person_node(self, boxes=True):
        node = person.YoloDetector.__new__(person.YoloDetector)
        node.image_lock = threading.Lock()
        node.result_image_index = 0
        node.latest_image = self.frame.copy()
        node.latest_header = Header()
        node.save_path = self.save_dir
        node.conf_thres = 0.75
        node.iou_thres = 0.45
        node.img_size = 640
        node.classes = None
        node.max_det = 1000
        node.line_thickness = 2
        node.publish_image = False
        node.view_image = False
        node.pred_pub = Mock()
        box = Mock()
        box.cls.item.return_value = 0
        box.conf.item.return_value = 0.9
        box.xyxy.cpu.return_value.numpy.return_value = np.array([[10, 10, 20, 40]])
        result = Mock()
        result.boxes = [box] if boxes else []
        result.plot.return_value = self.annotated.copy()
        node.model = Mock(return_value=[result])
        node.model.names = {0: "community"}
        return node

    def test_plate_requests_save_separate_annotated_images(self):
        node = self.plate_node()
        for _ in range(2):
            self.assertEqual(node.handle_recognition(Mock(detect_flag=3)).result, "ABC123")
        files = list(Path(self.save_dir).glob("plate_*.jpg"))
        self.assertEqual(sorted(path.name for path in files), ["plate_1.jpg", "plate_2.jpg"])
        for filename in files:
            np.testing.assert_array_equal(cv2.imread(str(filename)), self.annotated)

    def test_failed_plate_recognition_still_saves_result_image(self):
        node = self.plate_node()
        node.recognize_plate.return_value = None
        self.assertEqual(node.handle_recognition(Mock(detect_flag=3)).result,
                         "RECOGNITION_FAILED")
        files = list(Path(self.save_dir).glob("plate_*.jpg"))
        self.assertEqual(len(files), 1)
        self.assertGreater(np.count_nonzero(cv2.imread(str(files[0]))), 0)

    def test_person_requests_save_each_side_without_overwriting(self):
        node = self.person_node()
        for flag in (1, 2, 1):
            self.assertEqual(node.detect_callback(Mock(detect_flag=flag)).result,
                             "community:1")
        files = list(Path(self.save_dir).glob("person_*.jpg"))
        self.assertEqual(sorted(path.name for path in files),
                         ["person_1.jpg", "person_2.jpg", "person_3.jpg"])
        for filename in files:
            np.testing.assert_array_equal(cv2.imread(str(filename)), self.annotated)

    def test_no_person_detected_still_saves_image(self):
        node = self.person_node(boxes=False)
        self.assertEqual(node.detect_callback(Mock(detect_flag=1)).result, "")
        files = list(Path(self.save_dir).glob("person_1.jpg"))
        self.assertEqual(len(files), 1)
        self.assertIsNotNone(cv2.imread(str(files[0])))

    def test_no_camera_image_returns_error_without_inference(self):
        for node, handler, flag in (
                (self.plate_node(), "handle_recognition", 3),
                (self.person_node(), "detect_callback", 1)):
            with self.subTest(handler=handler):
                node.latest_image = None
                self.assertEqual(getattr(node, handler)(Mock(detect_flag=flag)).result,
                                 "ERROR: No image available")
        self.assertFalse(Path(self.save_dir).exists())

    def test_restarted_nodes_overwrite_numbered_images(self):
        for factory, handler, flag, filename in (
                (self.plate_node, "handle_recognition", 3, "plate_1.jpg"),
                (self.person_node, "detect_callback", 1, "person_1.jpg")):
            with self.subTest(filename=filename):
                getattr(factory(), handler)(Mock(detect_flag=flag))
                self.annotated[:] = 80
                getattr(factory(), handler)(Mock(detect_flag=flag))
                files = list(Path(self.save_dir).glob(filename.split("_")[0] + "_*.jpg"))
                self.assertEqual(len(files), 1)
                np.testing.assert_array_equal(
                    cv2.imread(str(Path(self.save_dir) / filename)), self.annotated)
                self.annotated[:] = 180

    def test_failed_image_write_is_logged_without_losing_recognition_result(self):
        for node, handler, flag, expected in (
                (self.plate_node(), "handle_recognition", 3, "ABC123"),
                (self.person_node(), "detect_callback", 1, "community:1")):
            with self.subTest(handler=handler), \
                 patch.object(cv2, "imwrite", return_value=False), \
                 patch.object(plate.rospy, "logerr") as logerr:
                self.assertEqual(getattr(node, handler)(Mock(detect_flag=flag)).result,
                                 expected)
                logerr.assert_called_once()


if __name__ == "__main__":
    unittest.main()
