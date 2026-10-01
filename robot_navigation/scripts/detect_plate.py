#!/usr/bin/env python3
"""Recognize the latest camera frame with Baidu's license plate OCR service."""

import base64
import os
import threading
import time
from urllib.parse import urlencode

import cv2
import numpy as np
import requests
import rospy
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as PILImage, ImageDraw, ImageFont
from sensor_msgs.msg import Image

from robot_navigation.srv import detect, detectResponse


ACCESS_TOKEN_URL = 'https://aip.baidubce.com/oauth/2.0/token'
PLATE_API_URL = 'https://aip.baidubce.com/rest/2.0/ocr/v1/license_plate'


def first_plate(result):
    """Return the first plate from either response shape observed in use."""
    plates = result.get('words_result')
    if isinstance(plates, dict):
        plates = [plates]
    if not isinstance(plates, list) or not plates:
        return None
    plate = plates[0]
    if not isinstance(plate, dict):
        return None
    number = plate.get('number')
    if not isinstance(number, str) or not number.strip():
        return None
    return number.strip(), plate.get('vertexes_location')


def load_credentials_file(path):
    """Read only the two expected keys from a simple KEY=VALUE file."""
    credentials = {}
    try:
        with open(path, encoding='utf-8') as stream:
            for line in stream:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                name, separator, value = line.partition('=')
                if separator and name.strip() in (
                        'BAIDU_OCR_API_KEY', 'BAIDU_OCR_SECRET_KEY'):
                    credentials[name.strip()] = value.strip().strip('"\'')
    except OSError:
        raise RuntimeError('无法读取百度 OCR 配置文件') from None
    return credentials


class PlateRecognizer:
    def __init__(self):
        rospy.init_node('plate_recognition_service')
        self.api_key = os.environ.get('BAIDU_OCR_API_KEY', '').strip()
        self.secret_key = os.environ.get('BAIDU_OCR_SECRET_KEY', '').strip()
        if not self.api_key or not self.secret_key:
            credentials_file = rospy.get_param('~credentials_file', '')
            if credentials_file:
                credentials = load_credentials_file(credentials_file)
                self.api_key = self.api_key or credentials.get('BAIDU_OCR_API_KEY', '')
                self.secret_key = self.secret_key or credentials.get('BAIDU_OCR_SECRET_KEY', '')
        if not self.api_key or not self.secret_key:
            raise RuntimeError('请配置百度 OCR 环境变量或凭据文件')

        self.bridge = CvBridge()
        self.latest_image = None
        self.image_lock = threading.Lock()
        self.image_topic = rospy.get_param('~image_topic', '/image_raw')
        self.save_dir = os.path.expanduser(
            rospy.get_param('~save_dir', '~/smart_ws/plate_samples'))
        self.font_path = rospy.get_param(
            '~font_path', '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc')
        self.access_token = self.get_access_token()

        self.image_sub = rospy.Subscriber(
            self.image_topic, Image, self.image_callback, queue_size=1)
        self.result_pub = rospy.Publisher('/recognized_image', Image, queue_size=1)
        self.service = rospy.Service('/recognize_plate', detect, self.handle_recognition)
        rospy.loginfo('车牌识别服务已就绪，使用话题 %s', self.image_topic)

    def image_callback(self, message):
        try:
            frame = self.bridge.imgmsg_to_cv2(message, 'bgr8')
        except CvBridgeError as exc:
            rospy.logerr('相机图像转换失败: %s', exc)
            return
        with self.image_lock:
            self.latest_image = frame.copy()

    def get_access_token(self):
        try:
            response = requests.post(
                ACCESS_TOKEN_URL,
                params={
                    'grant_type': 'client_credentials',
                    'client_id': self.api_key,
                    'client_secret': self.secret_key,
                },
                timeout=10,
            )
        except requests.RequestException as exc:
            # RequestException may contain the credential-bearing URL.
            raise RuntimeError('百度鉴权网络请求失败: {}'.format(type(exc).__name__)) from None
        if response.status_code != 200:
            raise RuntimeError('百度鉴权 HTTP 状态 {}'.format(response.status_code))
        try:
            data = response.json()
        except ValueError:
            raise RuntimeError('百度鉴权返回了非 JSON 内容') from None
        if not isinstance(data, dict):
            raise RuntimeError('百度鉴权返回格式错误')
        token = data.get('access_token')
        if not token:
            raise RuntimeError('百度鉴权失败，错误码 {}'.format(data.get('error', 'unknown')))
        return token

    def recognize_plate(self, frame):
        encoded, buffer = cv2.imencode('.jpg', frame)
        if not encoded:
            raise ValueError('相机帧无法编码为 JPEG')
        payload = {'image': base64.b64encode(buffer.tobytes()).decode('ascii')}
        if len(urlencode(payload).encode('ascii')) > 4 * 1024 * 1024:
            raise ValueError('图片编码后超过百度 OCR 的 4 MB 限制')
        headers = {'Content-Type': 'application/x-www-form-urlencoded'}
        for attempt in range(2):
            try:
                response = requests.post(
                    PLATE_API_URL,
                    params={'access_token': self.access_token},
                    data=payload,
                    headers=headers,
                    timeout=15,
                )
            except requests.RequestException as exc:
                raise RuntimeError('车牌 OCR 网络请求失败: {}'.format(type(exc).__name__)) from None
            if response.status_code != 200:
                raise RuntimeError('车牌 OCR HTTP 状态 {}'.format(response.status_code))
            try:
                result = response.json()
            except ValueError:
                raise RuntimeError('车牌 OCR 返回了非 JSON 内容') from None
            if not isinstance(result, dict):
                raise RuntimeError('车牌 OCR 返回格式错误')
            if result.get('error_code') in (110, 111) and attempt == 0:
                self.access_token = self.get_access_token()
                continue
            if 'error_code' in result:
                raise RuntimeError('车牌 OCR 错误码 {}'.format(result['error_code']))
            return result
        raise RuntimeError('车牌 OCR 访问令牌更新后仍不可用')

    def draw_result(self, frame, number, vertices):
        annotated = frame.copy()
        if not isinstance(vertices, list) or len(vertices) != 4:
            return annotated
        try:
            points = np.array(
                [[int(point['x']), int(point['y'])] for point in vertices],
                dtype=np.int32,
            )
        except (KeyError, TypeError, ValueError):
            return annotated
        cv2.polylines(annotated, [points], True, (0, 255, 0), 2)
        try:
            font = ImageFont.truetype(self.font_path, 30)
            image = PILImage.fromarray(cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB))
            ImageDraw.Draw(image).text(
                (int(points[0][0]), max(0, int(points[0][1]) - 35)),
                number, font=font, fill=(255, 255, 0),
            )
            return cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)
        except (OSError, UnicodeError):
            rospy.logwarn('中文字体不可用，仅保存车牌框；识别号码仍会输出')
            return annotated

    def handle_recognition(self, request):
        if request.detect_flag != 3:
            return detectResponse('ERROR: detect_flag 必须为 3')
        with self.image_lock:
            frame = None if self.latest_image is None else self.latest_image.copy()
        if frame is None:
            return detectResponse('ERROR: 尚未收到相机图像')

        try:
            result = self.recognize_plate(frame)
            plate = first_plate(result)
            if plate is None:
                return detectResponse('RECOGNITION_FAILED')
            number, vertices = plate
            annotated = self.draw_result(frame, number, vertices)
            os.makedirs(self.save_dir, exist_ok=True)
            path = os.path.join(
                self.save_dir, 'plate_detected_{}.png'.format(time.time_ns()))
            if not cv2.imwrite(path, annotated):
                raise OSError('识别结果图像写入失败')
            self.result_pub.publish(self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))
            rospy.loginfo('识别车牌：%s；结果图：%s', number, path)
            return detectResponse(number)
        except (RuntimeError, ValueError, OSError, cv2.error, CvBridgeError) as exc:
            rospy.logerr('车牌识别失败: %s', exc)
            return detectResponse('ERROR: {}'.format(exc))


if __name__ == '__main__':
    try:
        PlateRecognizer()
        rospy.spin()
    except (RuntimeError, rospy.ROSInterruptException) as exc:
        rospy.logerr('车牌识别节点启动失败: %s', exc)
