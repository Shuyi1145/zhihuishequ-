#!/usr/bin/env python3

import rospy
import cv2
import base64
import requests
from urllib3.util import ssl_ as urllib3_ssl

# 保留 HTTPS 证书校验，并兼容当前 urllib3 TLS 实现。
if urllib3_ssl.IS_PYOPENSSL:
    from urllib3.contrib import pyopenssl
    pyopenssl.extract_from_urllib3()

import threading
import os
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from robot_navigation.srv import *
from PIL import Image as PILImage, ImageDraw, ImageFont 
# 百度 API 配置。
API_KEY = "zLzrnJeQHcmQkkOYtM2DmVya"
SECRET_KEY = "SBmm9lwUd7L2HJGjBFVZlMzhpSJljM8u"
ACCESS_TOKEN_URL = "https://aip.baidubce.com/oauth/2.0/token"
PLATE_API_URL = "https://aip.baidubce.com/rest/2.0/ocr/v1/license_plate"

# 中文绘制配置，路径可通过 ROS 参数覆盖。
FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
FONT_SIZE = 20

class PlateRecognizer:
    def __init__(self):
        rospy.init_node('plate_recognition_service')
        self.font = self.load_font()
        
        # 注册识别服务。
        self.service = rospy.Service('/recognize_plate', detect, self.handle_recognition)
        
        # 订阅原始图像，并发布标注结果。
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_lock = threading.Lock()
        self.result_image_index = 0
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.image_callback)
        self.result_pub = rospy.Publisher("/recognized_image", Image, queue_size=1) 
        self.save_dir = rospy.get_param(
            '~save_dir', '/home/shuiy/catkin_ws/src/robot_navigation/scripts/pic')
        # 初始化访问令牌。
        self.access_token = self.get_access_token()
        if not self.access_token:
            rospy.logerr("Plate API token unavailable")
            rospy.signal_shutdown("API authentication failed")
            return
            
        rospy.loginfo("Plate recognition service ready")

    def load_font(self):
        """优先使用配置字体，再尝试系统中文字体。"""
        configured_path = rospy.get_param('~font_path', FONT_PATH)
        candidates = [configured_path, FONT_PATH,
                      '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc',
                      '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf']
        for font_path in dict.fromkeys(candidates):
            try:
                return ImageFont.truetype(font_path, FONT_SIZE)
            except (OSError, TypeError):
                continue
        rospy.logerr("No usable Chinese font found; plate text cannot be drawn.")
        return None

    def image_callback(self, image_msg):
        """保存最新图像。"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            with self.image_lock:
                self.latest_image = cv_image.copy()
        except Exception as exc:
            rospy.logerr("Image conversion failed: %s", exc)

    def get_access_token(self):
        """获取百度 API 访问令牌。"""
        params = {
            "grant_type": "client_credentials",
            "client_id": API_KEY,
            "client_secret": SECRET_KEY
        }
        try:
            response = requests.post(ACCESS_TOKEN_URL, params=params)
            response.raise_for_status()
            return response.json().get("access_token")
        except requests.exceptions.RequestException as exc:
            rospy.logerr("Plate API token request failed: %s", exc)
            return None

    def recognize_plate(self, cv_image):
        """调用百度 API，并返回完整的 JSON 结果。"""
        if cv_image is None:
            return None
            
        # 将图像编码为 JPEG Base64。
        _, buffer = cv2.imencode('.jpg', cv_image)
        img_base64 = base64.b64encode(buffer).decode('utf-8')
        
        payload = {
            'image': img_base64,
            'access_token': self.access_token
        }
        headers = {'Content-Type': 'application/x-www-form-urlencoded'}
        
        try:
            response = requests.post(PLATE_API_URL, headers=headers, data=payload)
            response.raise_for_status()
            return response.json()
        except Exception as exc:
            rospy.logerr("Plate API request failed: %s", exc)
            return None

    def draw_plate_info(self, image, result_json):
        """在图像上绘制车牌号和边框。"""
        draw_image = image.copy()
        
        if 'words_result' not in result_json:
            return draw_image, "RECOGNITION_FAILED_NO_WORDS"

        words_result = result_json['words_result']
        plate_number = words_result.get('number', 'N/A')
        location = words_result.get('vertexes_location', None)

        if location and len(location) == 4:
            points = np.array([[p['x'], p['y']] for p in location], np.int32)
            cv2.polylines(draw_image, [points], isClosed=True, color=(0, 255, 0), thickness=2)
            
            rospy.loginfo("Plate recognized: %s", plate_number)
        else:
            rospy.logwarn("Plate number recognized but vertexes_location not found or invalid.")

        # 文字固定在左上角，并添加描边提升可读性。
        if self.font is not None:
            pil_image = PILImage.fromarray(cv2.cvtColor(draw_image, cv2.COLOR_BGR2RGB))
            draw = ImageDraw.Draw(pil_image)
            draw.text((10, 10), plate_number, font=self.font, anchor="lt",
                      fill=(255, 255, 0), stroke_width=1, stroke_fill=(0, 0, 0))
            draw_image = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)

        return draw_image, plate_number

    def handle_recognition(self, req):
        """处理车牌识别请求。"""
        if req.detect_flag==3:
            rospy.loginfo("Plate detection started: flag=%d", req.detect_flag)
            # 复制最新图像，避免锁外继续使用共享数据。
            with self.image_lock:
                current_image = None if self.latest_image is None else self.latest_image.copy()
                
            if current_image is None:
                rospy.logwarn("No image available for plate detection")
                return detectResponse("ERROR: No image available")
                
            result_json = self.recognize_plate(current_image)
            
            if result_json:
                result_image, plate_number = self.draw_plate_info(current_image, result_json)
            else:
                rospy.logwarn("Plate recognition returned no result")
                result_image = current_image.copy()
                plate_number = "RECOGNITION_FAILED"

            # 按请求顺序保存结果图像。
            if not plate_number or plate_number == 'N/A':
                plate_number = "RECOGNITION_FAILED"
            if plate_number.startswith("RECOGNITION_FAILED"):
                cv2.putText(result_image, plate_number, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            try:
                os.makedirs(self.save_dir, exist_ok=True)
                with self.image_lock:
                    self.result_image_index += 1
                    image_index = self.result_image_index
                save_path = os.path.join(self.save_dir, "plate_%d.jpg" % image_index)
                if not cv2.imwrite(save_path, result_image):
                    raise OSError("cv2.imwrite returned False")
                rospy.loginfo("Plate image saved: %s", save_path)
            except (OSError, cv2.error) as exc:
                rospy.logerr("Plate image save failed: %s", exc)
            return detectResponse(plate_number)

if __name__ == '__main__':
    try:
        recognizer = PlateRecognizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
