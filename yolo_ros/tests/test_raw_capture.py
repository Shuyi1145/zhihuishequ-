"""Check A/B image capture filenames and write failures without ROS."""

import ast
import os
import pathlib
import tempfile
import unittest


SCRIPT = pathlib.Path(__file__).resolve().parents[1] / 'src' / 'detect_ros.py'


def load_save_capture(cv2, clock):
    source = ast.parse(SCRIPT.read_text(encoding='utf-8'))
    detector = next(node for node in source.body
                    if isinstance(node, ast.ClassDef) and node.name == 'YoloDetector')
    method = next(node for node in detector.body
                  if isinstance(node, ast.FunctionDef) and node.name == 'save_capture')
    namespace = {'os': os, 'cv2': cv2, 'time': clock}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[])),
                 str(SCRIPT), 'exec'), namespace)
    return namespace['save_capture']


class RawCaptureTest(unittest.TestCase):
    def test_each_area_saves_unique_raw_and_detected_images(self):
        class Cv2:
            error = RuntimeError

            @staticmethod
            def imwrite(path, frame):
                pathlib.Path(path).write_bytes(frame)
                return True

        class Clock:
            value = 0

            @classmethod
            def time_ns(cls):
                cls.value += 1
                return cls.value

        save = load_save_capture(Cv2, Clock)
        with tempfile.TemporaryDirectory() as directory:
            detector = type('Detector', (), {'capture_dir': directory})()
            a_raw, a_detected = save(detector, b'A source', b'A boxes', 1)
            b_raw, b_detected = save(detector, b'B source', b'B boxes', 2)
            self.assertEqual(pathlib.Path(a_raw).read_bytes(), b'A source')
            self.assertEqual(pathlib.Path(b_raw).read_bytes(), b'B source')
            self.assertEqual(pathlib.Path(a_detected).read_bytes(), b'A boxes')
            self.assertEqual(pathlib.Path(b_detected).read_bytes(), b'B boxes')
            self.assertEqual(len({a_raw, a_detected, b_raw, b_detected}), 4)
            self.assertTrue(pathlib.Path(a_raw).name.startswith('A_'))
            self.assertTrue(pathlib.Path(b_raw).name.startswith('B_'))

    def test_failed_raw_write_is_reported(self):
        class Cv2:
            error = RuntimeError

            @staticmethod
            def imwrite(path, frame):
                return False

        class Clock:
            @staticmethod
            def time_ns():
                return 1

        save = load_save_capture(Cv2, Clock)
        with tempfile.TemporaryDirectory() as directory:
            detector = type('Detector', (), {'capture_dir': directory})()
            with self.assertRaises(OSError):
                save(detector, b'raw', b'detected', 1)


if __name__ == '__main__':
    unittest.main()
