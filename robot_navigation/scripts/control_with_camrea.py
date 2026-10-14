#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import cv2
import rospy
import select
import termios
import tty

from datetime import datetime
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


msg = """
$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
通用机器人键盘控制 + 单相机拍照/录像
$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

%%%%%%%%%%%%%%%%%%%%%%%
移动控制（/cmd_vel）
%%%%%%%%%%%%%%%%%%%%%%%
        w
   a    s    d

w : 前进
s : 后退
a : 左转
d : 右转

---------------------------
速度调节
---------------------------
g : 降低线速度
h : 提高线速度
j : 降低角速度
l : 提高角速度

---------------------------
相机控制
---------------------------
p : 拍一张照片
v : 开始/停止录像
o : 打开/关闭预览窗口

---------------------------
其他
---------------------------
空格 : 急停
CTRL-C : 退出程序
"""


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class RobotCameraTeleop:
    def __init__(self):
        rospy.init_node("robot_camera_teleop", anonymous=True)

        # ========== 参数 ==========
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.camera_topic = rospy.get_param("~camera_topic", "/image_raw")

        self.linear_speed = rospy.get_param("~linear_speed", 0.3)
        self.angular_speed = rospy.get_param("~angular_speed", 0.8)

        self.linear_step = rospy.get_param("~linear_step", 0.05)
        self.angular_step = rospy.get_param("~angular_step", 0.1)

        self.max_linear_speed = rospy.get_param("~max_linear_speed", 1.5)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 3.0)

        self.enable_preview = rospy.get_param("~enable_preview", True)
        self.video_fps = rospy.get_param("~video_fps", 20.0)

        self.save_root = rospy.get_param("~save_root", "/home/shuiy/catkin_ws/src/ultralytics/raw_data/video")
        self.photo_dir = rospy.get_param("~photo_dir", os.path.join(self.save_root, "photos"))
        self.video_dir = rospy.get_param("~video_dir", os.path.join(self.save_root, "videos"))

        os.makedirs(self.photo_dir, exist_ok=True)
        os.makedirs(self.video_dir, exist_ok=True)

        # ========== ROS 通信 ==========
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.image_sub = rospy.Subscriber(self.camera_topic, Image, self.camera_callback)

        # ========== 相机相关 ==========
        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_frame_time = None

        self.recording = False
        self.video_writer = None
        self.current_video_path = None

        # ========== 运动控制 ==========
        self.twist = Twist()

        rospy.loginfo("robot_camera_teleop initialized.")
        rospy.loginfo("cmd_vel topic: %s", self.cmd_vel_topic)
        rospy.loginfo("camera topic: %s", self.camera_topic)
        rospy.loginfo("photo dir: %s", self.photo_dir)
        rospy.loginfo("video dir: %s", self.video_dir)
        rospy.loginfo("preview enabled: %s", str(self.enable_preview))

    def clamp(self, value, low, high):
        return max(low, min(value, high))

    def timestamp_str(self):
        return datetime.now().strftime("%Y%m%d_%H%M%S")

    def stop_robot(self):
        self.twist = Twist()
        self.cmd_vel_pub.publish(self.twist)

    def publish_motion(self, key):
        twist = Twist()

        if key in ['w', 'W']:
            twist.linear.x = self.linear_speed
        elif key in ['s', 'S']:
            twist.linear.x = -self.linear_speed
        elif key in ['a', 'A']:
            twist.angular.z = self.angular_speed
        elif key in ['d', 'D']:
            twist.angular.z = -self.angular_speed
        elif key == ' ':
            twist = Twist()

        self.twist = twist
        self.cmd_vel_pub.publish(self.twist)

    def adjust_speed(self, key):
        if key in ['h', 'H']:
            self.linear_speed = self.clamp(
                self.linear_speed + self.linear_step,
                0.0,
                self.max_linear_speed
            )
            print("[Speed] linear = {:.2f} m/s".format(self.linear_speed))

        elif key in ['g', 'G']:
            self.linear_speed = self.clamp(
                self.linear_speed - self.linear_step,
                0.0,
                self.max_linear_speed
            )
            print("[Speed] linear = {:.2f} m/s".format(self.linear_speed))

        elif key in ['l', 'L']:
            self.angular_speed = self.clamp(
                self.angular_speed + self.angular_step,
                0.0,
                self.max_angular_speed
            )
            print("[Speed] angular = {:.2f} rad/s".format(self.angular_speed))

        elif key in ['j', 'J']:
            self.angular_speed = self.clamp(
                self.angular_speed - self.angular_step,
                0.0,
                self.max_angular_speed
            )
            print("[Speed] angular = {:.2f} rad/s".format(self.angular_speed))

    def toggle_preview(self):
        self.enable_preview = not self.enable_preview
        print("[Camera] preview {}".format("enabled" if self.enable_preview else "disabled"))
        if not self.enable_preview:
            cv2.destroyAllWindows()

    def take_photo(self):
        if self.latest_frame is None:
            print("[Camera] no frame received yet, cannot take photo.")
            return

        filename = "{}.jpg".format(self.timestamp_str())
        filepath = os.path.join(self.photo_dir, filename)

        success = cv2.imwrite(filepath, self.latest_frame)
        if success:
            print("[Camera] photo saved: {}".format(filepath))
        else:
            print("[Camera] failed to save photo: {}".format(filepath))

    def start_recording(self):
        if self.recording:
            print("[Camera] already recording.")
            return

        if self.latest_frame is None:
            print("[Camera] no frame received yet, cannot start recording.")
            return

        h, w = self.latest_frame.shape[:2]
        filename = "{}.mp4".format(self.timestamp_str())
        self.current_video_path = os.path.join(self.video_dir, filename)

        self.video_writer = cv2.VideoWriter(
            self.current_video_path,
            cv2.VideoWriter_fourcc(*'mp4v'),
            self.video_fps,
            (w, h)
        )

        if not self.video_writer.isOpened():
            print("[Camera] failed to open VideoWriter: {}".format(self.current_video_path))
            self.video_writer = None
            self.current_video_path = None
            return

        self.recording = True
        print("[Camera] recording started: {}".format(self.current_video_path))

    def stop_recording(self):
        if not self.recording:
            return

        self.recording = False

        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None

        print("[Camera] recording stopped.")
        if self.current_video_path is not None:
            print("[Camera] video saved: {}".format(self.current_video_path))
        self.current_video_path = None

    def toggle_recording(self):
        if self.recording:
            self.stop_recording()
        else:
            self.start_recording()

    def camera_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridgeError: %s", e)
            return

        self.latest_frame = frame
        self.latest_frame_time = rospy.Time.now()

        if self.recording and self.video_writer is not None:
            self.video_writer.write(frame)

        if self.enable_preview:
            show_frame = frame.copy()

            if self.recording:
                cv2.putText(
                    show_frame,
                    "RECORDING",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 0, 255),
                    2
                )

            status_text = "lin:{:.2f}  ang:{:.2f}".format(
                self.linear_speed, self.angular_speed
            )
            cv2.putText(
                show_frame,
                status_text,
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

            cv2.imshow("Camera Preview", show_frame)
            cv2.waitKey(1)

    def handle_key(self, key):
        if key == '':
            self.stop_robot()
            return

        if key in ['w', 'W', 'a', 'A', 's', 'S', 'd', 'D', ' ']:
            self.publish_motion(key)
            return

        if key in ['g', 'G', 'h', 'H', 'j', 'J', 'l', 'L']:
            self.adjust_speed(key)
            return

        if key in ['p', 'P']:
            self.take_photo()
            return

        if key in ['v', 'V']:
            self.toggle_recording()
            return

        if key in ['o', 'O']:
            self.toggle_preview()
            return

        # 未定义按键时，默认停车
        self.stop_robot()

    def release_resources(self):
        self.stop_recording()
        self.stop_robot()
        cv2.destroyAllWindows()

    def main_loop(self, settings):
        print(msg)
        print("[Info] linear speed = {:.2f} m/s, angular speed = {:.2f} rad/s".format(
            self.linear_speed, self.angular_speed
        ))

        while not rospy.is_shutdown():
            key = get_key(settings)

            if key == '\x03':  # Ctrl-C
                break

            self.handle_key(key)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    controller = None

    try:
        controller = RobotCameraTeleop()
        controller.main_loop(settings)
    except rospy.ROSInterruptException:
        pass
    finally:
        if controller is not None:
            controller.release_resources()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)