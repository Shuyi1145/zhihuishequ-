"""Check OCR response parsing and request shape without ROS or network."""

import ast
import base64
import pathlib
import tempfile
import unittest
from urllib.parse import urlencode

import cv2
import numpy as np
from PIL import Image as PILImage, ImageDraw, ImageFont


SCRIPT = pathlib.Path(__file__).resolve().parents[1] / 'scripts' / 'detect_plate.py'
FONT = SCRIPT.parents[1] / 'fonts' / 'NotoSansSC.ttf'


def load_function(name, class_name=None, namespace=None):
    source = ast.parse(SCRIPT.read_text(encoding='utf-8'))
    nodes = source.body
    if class_name:
        nodes = next(node.body for node in nodes
                     if isinstance(node, ast.ClassDef) and node.name == class_name)
    function = next(node for node in nodes
                    if isinstance(node, ast.FunctionDef) and node.name == name)
    context = {} if namespace is None else dict(namespace)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[])),
                 str(SCRIPT), 'exec'), context)
    return context[name]


class PlateOcrTest(unittest.TestCase):
    def test_result_image_contains_number_and_box(self):
        draw_result = load_function('draw_result', 'PlateRecognizer', {
            'cv2': cv2, 'np': np, 'PILImage': PILImage,
            'ImageDraw': ImageDraw, 'ImageFont': ImageFont,
        })
        detector = type('Detector', (), {'font_path': str(FONT)})()
        frame = np.full((480, 640, 3), 255, dtype=np.uint8)
        vertices = [
            {'x': 214, 'y': 193}, {'x': 300, 'y': 193},
            {'x': 300, 'y': 220}, {'x': 214, 'y': 220},
        ]
        result = draw_result(detector, frame, '冀DSX888', vertices)
        yellow_text = np.all(result[:50] == (0, 255, 255), axis=2)
        self.assertGreater(np.count_nonzero(yellow_text), 50)
        self.assertTrue(np.array_equal(result[193, 214], (0, 255, 0)))
        self.assertTrue(np.all(frame == 255))

    def test_credentials_file_reads_expected_keys(self):
        load_credentials_file = load_function('load_credentials_file')
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / 'baidu_ocr.env'
            path.write_text(
                '# OCR test configuration\n'
                "BAIDU_OCR_API_KEY='sample-key'\n"
                'BAIDU_OCR_SECRET_KEY=sample-secret\n'
                'IGNORED=value\n', encoding='utf-8')
            self.assertEqual(load_credentials_file(path), {
                'BAIDU_OCR_API_KEY': 'sample-key',
                'BAIDU_OCR_SECRET_KEY': 'sample-secret',
            })

    def test_baidu_response_list_and_observed_object(self):
        first_plate = load_function('first_plate')
        self.assertEqual(first_plate({'words_result': [
            {'number': '测试12345', 'vertexes_location': [{'x': 1, 'y': 2}]}]}),
            ('测试12345', [{'x': 1, 'y': 2}]))
        self.assertIsNone(first_plate({'words_result': []}))
        self.assertIsNone(first_plate({'error_code': 110}))
        self.assertEqual(first_plate({'words_result': {
            'number': '冀DSX888',
            'vertexes_location': [{'x': 214, 'y': 193}],
            'color': 'blue',
        }}), ('冀DSX888', [{'x': 214, 'y': 193}]))

    def test_token_is_query_parameter_and_image_is_form_body(self):
        calls = []

        class Buffer:
            @staticmethod
            def tobytes():
                return b'image bytes'

        class Cv2:
            @staticmethod
            def imencode(extension, frame):
                self.assertEqual(extension, '.jpg')
                return True, Buffer()

        class Response:
            status_code = 200

            @staticmethod
            def json():
                return {'words_result': [{'number': '测试12345'}]}

        class Requests:
            RequestException = RuntimeError

            @staticmethod
            def post(url, **kwargs):
                calls.append((url, kwargs))
                return Response()

        recognize = load_function('recognize_plate', 'PlateRecognizer', {
            'cv2': Cv2, 'requests': Requests, 'base64': base64,
            'urlencode': urlencode, 'PLATE_API_URL': 'https://example.test/plate'})
        detector = type('Detector', (), {'access_token': 'test-token'})()
        result = recognize(detector, object())
        self.assertEqual(result['words_result'][0]['number'], '测试12345')
        self.assertEqual(calls[0][1]['params'], {'access_token': 'test-token'})
        self.assertEqual(calls[0][1]['data'], {
            'image': base64.b64encode(b'image bytes').decode('ascii')})


if __name__ == '__main__':
    unittest.main()
