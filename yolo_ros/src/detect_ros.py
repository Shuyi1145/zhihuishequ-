#!/usr/bin/env python3

import rospy
import cv2
import torch
import numpy as np
import threading
import os

from ultralytics import YOLO
from cv_bridge import CvBridge
from rostopic import get_topic_type
from robot_navigation.srv import * 
from sensor_msgs.msg import Image, CompressedImage
from yolo_ros.msg import BoundingBox, BoundingBoxes
from std_msgs.msg import Header

class YoloDetector:
    def __init__(self):
        # 读取检测参数。
        weights_path = rospy.get_param("~weights")
        self.conf_thres = rospy.get_param("~confidence_threshold", 0.45)
        self.iou_thres = rospy.get_param("~iou_threshold", 0.5)
        self.max_det = rospy.get_param("~maximum_detections", 300)
        self.classes = rospy.get_param("~classes", None)  # 例如 [0, 1, 2]
        self.device = str(rospy.get_param("~device", "cpu"))
        self.img_size = rospy.get_param("~inference_size", 640)
        
        # 配置可视化输出。
        self.view_image = rospy.get_param("~view_image", True)
        self.publish_image = rospy.get_param("~publish_image", True)
        self.line_thickness = rospy.get_param("~line_thickness", 2)
        
        try:
            self.model = YOLO(weights_path)
            self.model.to(self.device)
            rospy.loginfo("YOLO model loaded successfully from %s", weights_path)
        except Exception as exc:
            rospy.logerr("YOLO model load failed: %s", exc)
            rospy.signal_shutdown("YOLO model load failed: %s" % exc)
            return
            
        # 保存最新图像，供服务请求使用。
        self.save_path = rospy.get_param(
            "~save_dir", '/home/shuiy/catkin_ws/src/robot_navigation/scripts/pic/')
        self.latest_image = None
        self.latest_header = Header()
        self.image_lock = threading.Lock()
        self.result_image_index = 0
        
        self.bridge = CvBridge()

        # 根据输入话题类型选择订阅消息。
        input_image_topic = rospy.get_param("~input_image_topic", "/camera/color/image_raw")
        input_image_type, _, _ = get_topic_type(input_image_topic, blocking=True)
        self.compressed_input = "CompressedImage" in input_image_type

        if self.compressed_input:
            self.image_sub = rospy.Subscriber(
                input_image_topic, CompressedImage, self.image_callback, queue_size=1
            )
        else:
            self.image_sub = rospy.Subscriber(
                input_image_topic, Image, self.image_callback, queue_size=1
            )
        
        self.pred_pub = rospy.Publisher(
            rospy.get_param("~output_topic", "/yolo/detections"), BoundingBoxes, queue_size=10
        )
        if self.publish_image:
            self.image_pub = rospy.Publisher(
                rospy.get_param("~output_image_topic", "/yolo/detection_image"), Image, queue_size=10
            )

        self.service = rospy.Service('/recognize_person', detect, self.detect_callback)
        rospy.loginfo("YOLO service ready: /recognize_person")

    def image_callback(self, image_msg):
        """保存最新图像及其头信息。"""
        try:
            if self.compressed_input:
                cv_image = self.bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")
            else:
                cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            with self.image_lock:
                self.latest_image = cv_image.copy()
                self.latest_header = image_msg.header
                
            if self.view_image:
                # 原始图像在这里预览，标注图像在服务回调中显示。
                cv2.imshow("YOLO Live View", cv_image)
                cv2.waitKey(1)

        except Exception as exc:
            rospy.logerr("Image conversion failed: %s", exc)

    def detect_callback(self, req):
        """执行检测，并返回各类别的数量。"""
        if req.detect_flag == 1 or req.detect_flag == 2:
            rospy.loginfo("YOLO detection started")
            
            # 复制一份图像，避免锁持有期间执行推理。
            with self.image_lock:
                if self.latest_image is not None:
                    im0 = self.latest_image.copy()
                    current_header = self.latest_header
                else:
                    rospy.logwarn("No image available for detection.")
                    return detectResponse("ERROR: No image available")

            inference_results = self.model(
                im0,
                conf=self.conf_thres,
                iou=self.iou_thres,
                imgsz=self.img_size,
                classes=self.classes,
                max_det=self.max_det,
                verbose=False
            )
            
            inference_result = inference_results[0]

            bounding_boxes_msg = BoundingBoxes()
            bounding_boxes_msg.header = current_header
            bounding_boxes_msg.image_header = current_header

            class_counts = {}

            if len(inference_result.boxes) > 0:
                rospy.loginfo("Detected objects: %d", len(inference_result.boxes))
                for box in inference_result.boxes:
                    bbox_msg = BoundingBox()
                    
                    cls_id = int(box.cls.item())
                    class_name = self.model.names[cls_id]
                    
                    class_counts[class_name] = class_counts.get(class_name, 0) + 1
                    
                    xyxy = box.xyxy.cpu().numpy().flatten().astype(int)
                    conf = box.conf.item()
                    
                    bbox_msg.Class = class_name
                    bbox_msg.probability = conf
                    bbox_msg.xmin = xyxy[0]
                    bbox_msg.ymin = xyxy[1]
                    bbox_msg.xmax = xyxy[2]
                    bbox_msg.ymax = xyxy[3]
                    
                    bounding_boxes_msg.bounding_boxes.append(bbox_msg)
            else:
                rospy.loginfo("No objects detected")

            self.pred_pub.publish(bounding_boxes_msg)

            annotated_frame = inference_result.plot(line_width=self.line_thickness)
            if not class_counts:
                cv2.putText(annotated_frame, "No objects detected", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            # 按请求顺序编号，节点重启后从头开始。
            try:
                os.makedirs(self.save_path, exist_ok=True)
                with self.image_lock:
                    self.result_image_index += 1
                    image_index = self.result_image_index
                filename = "person_%d.jpg" % image_index
                save_path = os.path.join(self.save_path, filename)
                if not cv2.imwrite(save_path, annotated_frame):
                    raise OSError("cv2.imwrite returned False")
                rospy.loginfo("Detection image saved: %s", save_path)
            except (OSError, cv2.error) as exc:
                rospy.logerr("Detection image save failed: %s", exc)
            
            if self.publish_image:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8"))

            if self.view_image:
                cv2.imshow("YOLO Detection Result", annotated_frame)
                cv2.waitKey(1)
                
            # 将类别计数整理为稳定排序的响应字符串。
            response_items = [f"{cls}:{count}" for cls, count in class_counts.items()]
            response_str = ", ".join(sorted(response_items))
            
            rospy.loginfo("YOLO detection finished: %s", response_str or "None")
            
            return detectResponse(response_str)

if __name__ == "__main__":
    rospy.init_node("yolo_detector_node", anonymous=True)
    try:
        detector = YoloDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        # 退出时关闭 OpenCV 窗口。
        cv2.destroyAllWindows()
