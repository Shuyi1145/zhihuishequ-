#!/usr/bin/env python3
"""Example mission-side polling. Exit does not move the robot."""
import json
import time
import rospy
from std_srvs.srv import Trigger


def main():
    rospy.init_node('wait_for_green')
    timeout = float(rospy.get_param('~timeout', 120))
    if timeout <= 0:
        raise ValueError('timeout must be positive')
    rospy.wait_for_service('/traffic_light/check', timeout=10)
    rospy.wait_for_service('/traffic_light/reset', timeout=10)
    rospy.ServiceProxy('/traffic_light/reset', Trigger)()
    check = rospy.ServiceProxy('/traffic_light/check', Trigger)
    deadline = time.monotonic()+timeout
    labels = {'red': '红', 'yellow': '黄', 'green': '绿', 'unknown': '未知'}
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        try:
            response = check()
            result = json.loads(response.message)
            state = result.get('state', 'unknown')
            allowed = response.success and result.get('allowed') is True and state == 'green'
        except (rospy.ServiceException, ValueError) as exc:
            rospy.logwarn('查询失败，保持等待: %s', exc)
            state, allowed = 'unknown', False
        print('当前是：{}灯'.format(labels.get(state, '未知')), flush=True)
        print('通行：'+('可通行' if allowed else '不可通行'), flush=True)
        if allowed:
            print('通过中.....。结束检测（示例客户端结束，未发送运动指令）', flush=True)
            return 0
        print('等待中.....。', flush=True)
        time.sleep(0.2)
    print('等待超时或节点关闭，保持停车；本任务失败。', flush=True)
    return 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (rospy.ROSException, rospy.ROSInterruptException, rospy.ServiceException) as exc:
        print('ROS 调用失败，未允许通行：{}'.format(exc), flush=True)
        raise SystemExit(1)
