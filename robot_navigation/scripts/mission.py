#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from math import radians, pi
from std_srvs.srv import Empty
from robot_navigation.srv import *

class MoveBaseSquare():

    def __init__(self):
        rospy.init_node('misson', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        # 创建一个列表，保存目标的角度数据
        quaternions = list()        
    
        # 定义四个顶角处机器人的方向角度（Euler angles:http://zh.wikipedia.org/wiki/%E6%AC%A7%E6%8B%89%E8%A7%92)
        euler_angles = (0, 0, pi/2, pi/2,-pi/2,pi/2,pi/2, pi, pi,-pi/2,pi )

        # 将上面的Euler angles转换成Quaternion的格式
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            quaternions.append(q)
      
        # 创建一个列表存储导航点的位置
        waypoints = list()
        waypoints.append(Pose(Point(0.4 ,0.4 ,0), quaternions[0]))  

        waypoints.append(Pose(Point(3.831, 0.489, 0), quaternions[1]))      
        waypoints.append(Pose(Point(3.670, 1.572, 0), quaternions[2]))    
        waypoints.append(Pose(Point(2.990, 1.330, 0), quaternions[3])) 

        waypoints.append(Pose(Point(2.990, 1.997, 0), quaternions[4])) 
   
        waypoints.append(Pose(Point(2.185, 1.881, 0), quaternions[5]))   
        waypoints.append(Pose(Point(2.210, 3.664, 0), quaternions[6]))  
        waypoints.append(Pose(Point(1.068, 3.839, 0), quaternions[7]))
        waypoints.append(Pose(Point(1.190, 3.088, 0), quaternions[8])) 
        waypoints.append(Pose(Point(0.865, 0.388, 0), quaternions[9]))    
        waypoints.append(Pose(Point(0.516, 0.468, 0), quaternions[10]))



        self.count = 1
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist,queue_size=10) 
        self.stop_flag=0
        # 订阅move_base服务器的消息
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        client_person=rospy.ServiceProxy("recognize_person",detect)
        client_plate=rospy.ServiceProxy("recognize_plate",detect)
        rospy.loginfo("mission start!")
    
        while(not rospy.is_shutdown()):
            while(self.stop_flag!=1):
                rospy.loginfo("正在前往目标点%d",self.count)
                # 初始化goal为MoveBaseGoal类型
                goal = MoveBaseGoal()  
                # 使用map的frame定义goal的frame id
                goal.target_pose.header.frame_id = 'map'
                # 设置时间戳
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = waypoints[self.count]        
                if(self.move(goal) == True):
                    self.clear_costmaps_service()   #清除产生的偏移代价地图
                    self.count += 1 
                    if(self.count==11):
                        self.stop_flag=1
                    elif(self.count==4 ):
                        person_result=client_person.call(1)
                        rospy.sleep(1)   
                        rospy.loginfo("第一处社区识别到：%s",person_result)  
                    elif(self.count==5 ):
                        person_result=client_person.call(2)
                        rospy.sleep(1)   
                        rospy.loginfo("第二处社区识别到：%s",person_result)  
                    elif(self.count==9):
                        plate_result=client_plate.call(3) 
                        rospy.sleep(1)     
                        rospy.loginfo("识别到的车牌号为：%s",plate_result)   
                    

            
    def move(self, goal):
            # 把目标位置发送给MoveBaseAction的服务器
            self.move_base.send_goal(goal)
            # 设定1分钟的时间限制
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(60))
            # 如果一分钟之内没有到达，放弃目标
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    return True
            return False        
    
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

  
if __name__ == '__main__':
    try:
        MoveBaseSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
