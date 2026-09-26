#!/usr/bin/env python3
"""Persistent image subscriber; query service never commands robot motion."""
from dataclasses import fields
import json
import threading
import time

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

from traffic_light_detector.detector import Settings, detect
from traffic_light_detector.gate import GreenGate


class TrafficLightNode:
    def __init__(self):
        self.lock = threading.RLock()
        self.bridge = CvBridge()
        self.latest = None
        self.last_stamp = None
        self.reason = '尚未收到图像'
        self.result_stamp = None
        self.frequency = float(rospy.get_param('~detection_rate', 5.0))
        self.timeout = float(rospy.get_param('~image_timeout', 1.0))
        if self.frequency <= 0:
            raise ValueError('detection_rate must be positive')
        self.gate = GreenGate(float(rospy.get_param('~green_hold', 0.5)),
                              int(rospy.get_param('~green_min_frames', 3)), self.timeout)
        defaults = Settings()
        self.settings = Settings(**{field.name: rospy.get_param('~'+field.name, getattr(defaults, field.name))
                                    for field in fields(Settings)})
        if not 0 < self.settings.roi_bottom <= 1:
            raise ValueError('roi_bottom must be in (0,1]')
        self.state_pub = rospy.Publisher('~state', String, queue_size=1)
        self.image_pub = rospy.Publisher('~image', Image, queue_size=1)
        self.service = rospy.Service('~check', Trigger, self.check)
        self.reset_service = rospy.Service('~reset', Trigger, self.reset)
        self.sub = rospy.Subscriber(rospy.get_param('~image_topic', '/image_raw'), Image,
                                    self.receive, queue_size=1, buff_size=2**24)

    def receive(self, message):
        stamp = message.header.stamp.to_sec()
        with self.lock:
            if stamp <= 0:
                self.gate.reset()
                self.latest = None
                self.reason = '图像缺少有效时间戳'
                return
            if self.last_stamp is not None and stamp <= self.last_stamp:
                if stamp < self.last_stamp:
                    self.gate.reset()
                    self.latest = None
                    self.result_stamp = None
                else:
                    return  # 重复帧不计入绿灯确认。
            self.last_stamp = stamp
            self.latest = (message, time.monotonic())

    def status(self):
        with self.lock:
            state, allowed = self.gate.snapshot(time.monotonic())
            if self.result_stamp is not None:
                age = rospy.Time.now().to_sec()-self.result_stamp
                if age < 0 or age > self.timeout:
                    state, allowed = 'unknown', False
            reason = self.reason if state != 'unknown' else '图像无效、结果未知或已过期；'+self.reason
            return {'state': state, 'allowed': allowed, 'reason': reason}

    def check(self, _request):
        result = self.status()
        return TriggerResponse(success=result['allowed'], message=json.dumps(result, ensure_ascii=False))

    def reset(self, _request):
        with self.lock:
            self.gate.reset()
            self.latest = None
            self.result_stamp = None
            self.reason = '新观察任务，等待新图像'
        return TriggerResponse(success=True, message='已清空灯色确认，等待新图像')

    def run(self):
        previous = None
        while not rospy.is_shutdown():
            # 用墙钟轮询，Gazebo 暂停时也能将旧结果判为过期。
            with self.lock:
                frame, self.latest = self.latest, None
                if frame is not None:
                    message, received = frame
                    age = rospy.Time.now().to_sec()-message.header.stamp.to_sec()
                    try:
                        if time.monotonic()-received > self.timeout or not 0 <= age <= self.timeout:
                            raise ValueError('收到过期图像')
                        image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
                        result, overlay = detect(image, self.settings)
                        self.gate.update(result['state'], received)
                        self.result_stamp = message.header.stamp.to_sec()
                        self.reason = result['reason']
                        annotated = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
                        annotated.header = message.header
                        self.image_pub.publish(annotated)
                    except Exception as exc:
                        self.gate.reset()
                        self.reason = str(exc)
                        rospy.logwarn_throttle(5, '图像检测失败: %s', exc)
            result = self.status()
            self.state_pub.publish(String(data=json.dumps(result, ensure_ascii=False)))
            changed = (result['state'], result['allowed'])
            if changed != previous:
                rospy.loginfo('灯色: %s, 可通行: %s', *changed)
                previous = changed
            time.sleep(1.0/self.frequency)


if __name__ == '__main__':
    rospy.init_node('traffic_light')
    TrafficLightNode().run()
