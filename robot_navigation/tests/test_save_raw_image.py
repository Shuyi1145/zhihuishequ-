"""Check the one-frame camera capture without a ROS installation."""

import argparse
import ast
from datetime import datetime
from pathlib import Path
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'save_raw_image.py'


def load_main(namespace):
    source = ast.parse(SCRIPT.read_text(encoding='utf-8'))
    function = next(node for node in source.body
                    if isinstance(node, ast.FunctionDef) and node.name == 'main')
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[])),
                 str(SCRIPT), 'exec'), namespace)
    return namespace['main']


class CaptureTest(unittest.TestCase):
    def test_next_raw_frame_is_saved_as_png(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'plate_raw.png'
            calls = []

            class Cv2:
                error = RuntimeError

                @staticmethod
                def imwrite(path, frame):
                    calls.append(('write', path))
                    Path(path).write_bytes(frame)
                    return True

            class Ros:
                ROSException = RuntimeError

                @staticmethod
                def myargv(argv):
                    return ['save_raw_image.py', '--output', str(output)]

                @staticmethod
                def init_node(name, anonymous):
                    calls.append(('node', name))

                @staticmethod
                def wait_for_message(topic, message_type, timeout):
                    calls.append(('topic', topic))
                    return object()

            class Bridge:
                @staticmethod
                def imgmsg_to_cv2(message, encoding):
                    self.assertEqual(encoding, 'bgr8')
                    return b'camera pixels'

            main = load_main({
                'argparse': argparse, 'datetime': datetime, 'Path': Path,
                'sys': sys, 'cv2': Cv2, 'rospy': Ros,
                'CvBridge': Bridge, 'CvBridgeError': RuntimeError,
                'Image': object,
            })
            self.assertEqual(main(), 0)
            self.assertEqual(output.read_bytes(), b'camera pixels')
            self.assertIn(('topic', '/image_raw'), calls)


if __name__ == '__main__':
    unittest.main()
