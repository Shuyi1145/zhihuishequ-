#!/usr/bin/env python3
"""Save one unannotated ROS camera frame as a lossless PNG."""

import argparse
from datetime import datetime
from pathlib import Path
import sys

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser(description='保存一张 /image_raw 相机原图')
    parser.add_argument('--topic', default='/image_raw', help='ROS 图像话题')
    parser.add_argument('--output', help='输出 PNG 的完整文件路径')
    parser.add_argument('--timeout', type=float, default=10.0,
                        help='等待下一帧的秒数，默认 10 秒')
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    if args.timeout <= 0:
        parser.error('--timeout 必须大于零')

    rospy.init_node('save_raw_image', anonymous=True)
    try:
        message = rospy.wait_for_message(args.topic, Image, timeout=args.timeout)
        frame = CvBridge().imgmsg_to_cv2(message, 'bgr8')
        filename = args.output or str(
            Path.home() / 'smart_ws' / 'plate_samples' /
            'plate_raw_{}.png'.format(datetime.now().strftime('%Y%m%d_%H%M%S_%f')))
        path = Path(filename).expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(path), frame):
            raise OSError('图像写入失败')
    except (rospy.ROSException, CvBridgeError, OSError, cv2.error) as exc:
        rospy.logerr('采集相机原图失败: %s', exc)
        return 1
    print('已保存未标注原图：{}'.format(path))
    return 0


if __name__ == '__main__':
    sys.exit(main())
