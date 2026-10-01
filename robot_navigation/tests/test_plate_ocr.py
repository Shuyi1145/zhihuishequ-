"""Check OCR response parsing and request shape without ROS or network."""

import ast
import base64
import pathlib
import unittest
from urllib.parse import urlencode


SCRIPT = pathlib.Path(__file__).resolve().parents[1] / 'scripts' / 'detect_plate.py'


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
