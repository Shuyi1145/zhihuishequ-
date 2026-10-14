#!/usr/bin/env python3
"""使用机器人相机图像识别 Gazebo 红绿灯的当前亮灯颜色。

默认订阅 /image_raw（sensor_msgs/Image），逐帧发布 /traffic_light/state
（std_msgs/String：red、yellow、green 或 unknown），同时提供
/recognize_traffic_light（detect.srv，detect_flag=4）按需识别。
可用 ~roi 限定检测区域，需要查看检测框时设置 ~publish_debug:=true。
"""

import math
import os
import threading

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from robot_navigation.srv import detect, detectResponse


# OpenCV Hue 范围为 0-179；红色需要两个区间。
HUE_RANGES = {
    "red": ((0, 12), (170, 179)),
    "yellow": ((15, 40),),
    "green": ((42, 95),),
}
KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))


def roi_bounds(width, height, roi):
    """把归一化 ROI 转成像素边界，并校验范围。"""
    if not isinstance(roi, (list, tuple)) or len(roi) != 4:
        raise ValueError("roi must contain [x_min, y_min, x_max, y_max]")
    x_min, y_min, x_max, y_max = (float(value) for value in roi)
    if not all(math.isfinite(value) for value in (x_min, y_min, x_max, y_max)):
        raise ValueError("roi values must be finite")
    if not (0 <= x_min < x_max <= 1 and 0 <= y_min < y_max <= 1):
        raise ValueError("roi values must satisfy 0 <= min < max <= 1")
    return (int(x_min * width), int(y_min * height),
            max(1, math.ceil(x_max * width)), max(1, math.ceil(y_max * height)))


def largest_lamp(mask, min_pixels):
    """选取面积最大的近圆形亮区，过滤反光和零散噪声。"""
    count, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    best_area, best_box = 0, None
    for index in range(1, count):
        x, y, width, height, area = stats[index]
        if area < min_pixels or not 0.55 <= width / height <= 1.8:
            continue
        if area / (width * height) < 0.25:
            continue
        if area > best_area:
            best_area, best_box = int(area), (int(x), int(y), int(width), int(height))
    return best_area, best_box


def detect_light(frame, roi=(0, 0, 1, 1), min_pixels=12,
                 min_saturation=110, min_value=200):
    """返回颜色和检测框，检测框使用原图像素坐标。"""
    if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("frame must be a BGR image")
    if min_pixels < 1 or not 0 <= min_saturation <= 255 or not 0 <= min_value <= 255:
        raise ValueError("invalid HSV or min_pixels threshold")

    x_min, y_min, x_max, y_max = roi_bounds(frame.shape[1], frame.shape[0], roi)
    hsv = cv2.cvtColor(frame[y_min:y_max, x_min:x_max], cv2.COLOR_BGR2HSV)
    candidates = []
    for color, ranges in HUE_RANGES.items():
        mask = None
        for hue_min, hue_max in ranges:
            band = cv2.inRange(hsv, (hue_min, min_saturation, min_value),
                               (hue_max, 255, 255))
            mask = band if mask is None else cv2.bitwise_or(mask, band)
        # 合并一盏灯内分散的 LED 亮点，同时保留远处的小灯面。
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)
        area, box = largest_lamp(mask, min_pixels)
        if box is not None:
            x, y, width, height = box
            candidates.append((area, color, (x + x_min, y + y_min, width, height)))

    if not candidates:
        return "unknown", None
    candidates.sort(reverse=True)
    # 红灯的白色 LED 也可能落在黄色区间；只接受优势明显的颜色。
    if len(candidates) > 1 and candidates[1][0] >= candidates[0][0] * 0.8:
        return "unknown", None
    _, color, box = candidates[0]
    return color, box


class TrafficLightDetector:
    """订阅相机并按帧发布灯态；可选发布带检测框的调试图。"""

    def __init__(self):
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_lock = threading.Lock()
        self.result_image_index = 0
        # 红灯等待期间服务会重复轮询，只保留该次等待序列的首帧。
        self._red_sequence_saved = False
        self.save_dir = rospy.get_param(
            "~save_dir", "/home/shuiy/catkin_ws/src/robot_navigation/scripts/pic")
        self.roi = rospy.get_param("~roi", [0, 0, 1, 1])
        roi_bounds(640, 480, self.roi)
        self.min_pixels = int(rospy.get_param("~min_pixels", 12))
        self.min_saturation = int(rospy.get_param("~min_saturation", 110))
        self.min_value = int(rospy.get_param("~min_value", 200))
        if (self.min_pixels < 1 or not 0 <= self.min_saturation <= 255 or
                not 0 <= self.min_value <= 255):
            raise ValueError("invalid HSV or min_pixels threshold")

        image_topic = rospy.get_param("~image_topic", "/image_raw")
        state_topic = rospy.get_param("~state_topic", "/traffic_light/state")
        self.state_pub = rospy.Publisher(state_topic, String, queue_size=1)
        self.debug_pub = None
        if rospy.get_param("~publish_debug", False):
            debug_topic = rospy.get_param("~debug_image_topic",
                                          "/traffic_light/debug_image")
            self.debug_pub = rospy.Publisher(debug_topic, Image, queue_size=1)
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback,
                                          queue_size=1, buff_size=2 ** 22)
        self.service = rospy.Service("/recognize_traffic_light", detect,
                                     self.handle_recognition)
        rospy.loginfo("Traffic light detector ready: %s -> %s", image_topic, state_topic)

    def handle_recognition(self, req):
        if req.detect_flag != 4:
            return detectResponse("UNSUPPORTED_FLAG")

        with self.image_lock:
            frame = None if self.latest_image is None else self.latest_image.copy()
        if frame is None:
            return detectResponse("unknown")

        try:
            state, box = detect_light(frame, self.roi, self.min_pixels,
                                      self.min_saturation, self.min_value)
        except (cv2.error, ValueError) as exc:
            rospy.logwarn_throttle(5.0, "Traffic light detection failed: %s", exc)
            return detectResponse("unknown")

        # mission.py 在红灯期间会重复调用服务；一次红灯等待序列只保存首帧。
        with self.image_lock:
            red_sequence_saved = getattr(self, "_red_sequence_saved", False)
            save_result = not red_sequence_saved
            if state == "red":
                self._red_sequence_saved = True
            else:
                # 绿灯或黄灯结束红灯等待，下一次独立识别可以重新保存。
                self._red_sequence_saved = False
                if red_sequence_saved:
                    save_result = False

        if box is not None:
            x, y, width, height = box
            cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 255, 255), 2)
        cv2.putText(frame, state, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(frame, state, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 255, 255), 2, cv2.LINE_AA)
        if save_result:
            try:
                os.makedirs(self.save_dir, exist_ok=True)
                with self.image_lock:
                    self.result_image_index += 1
                    image_index = self.result_image_index
                save_path = os.path.join(
                    self.save_dir, "traffic_light_%d.jpg" % image_index)
                if not cv2.imwrite(save_path, frame):
                    raise OSError("cv2.imwrite returned False")
                rospy.loginfo("Traffic light image saved: %s", save_path)
            except (OSError, cv2.error) as exc:
                rospy.logerr("Traffic light image save failed: %s", exc)
        return detectResponse(state)

    def image_callback(self, msg):
        """每帧只做一次 HSV 转换；异常帧发布 unknown，避免保留旧灯态。"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            state, box = detect_light(frame, self.roi, self.min_pixels,
                                      self.min_saturation, self.min_value)
            with self.image_lock:
                self.latest_image = frame.copy()
        except (CvBridgeError, cv2.error, ValueError) as exc:
            rospy.logwarn_throttle(5.0, "Traffic light image conversion failed: %s", exc)
            with self.image_lock:
                self.latest_image = None
            self.state_pub.publish(String(data="unknown"))
            return

        self.state_pub.publish(String(data=state))
        if self.debug_pub is not None and self.debug_pub.get_num_connections():
            try:
                debug = frame.copy()
                if box is not None:
                    x, y, width, height = box
                    cv2.rectangle(debug, (x, y), (x + width, y + height),
                                  (0, 255, 255), 2)
                cv2.putText(debug, state, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (0, 0, 0), 4, cv2.LINE_AA)
                cv2.putText(debug, state, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (0, 255, 255), 2, cv2.LINE_AA)
                image = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
                image.header = msg.header
                self.debug_pub.publish(image)
            except (CvBridgeError, cv2.error) as exc:
                rospy.logwarn_throttle(5.0, "Traffic light debug image failed: %s", exc)


if __name__ == "__main__":
    rospy.init_node("traffic_light_detector")
    try:
        TrafficLightDetector()
    except ValueError as exc:
        rospy.logfatal("Traffic light detector configuration failed: %s", exc)
        raise SystemExit(1)
    rospy.spin()
