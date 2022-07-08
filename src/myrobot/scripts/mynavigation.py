#! /usr/bin/env python
#-*- coding:utf-8   -*-

import roslib
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt

# 引入照相
from take_a_photo import takePhoto
from object_detect import *

class NavTest():
    def __init__(self):
        rospy.init_node('random_navigation', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        # 在每个目标位置暂停的时间
        self.rest_time = rospy.get_param("~rest_time", 2)

        # 到达目标的状态
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',
                        'SUCCEEDED', 'ABORTED', 'REJECTED',
                        'PREEMPTING', 'RECALLING', 'RECALLED',
                        'LOST']

        # 设置目标点的位置
        locations = dict()

        locations['A'] = Pose(Point(5.246, -4.153, 0.000), Quaternion(0.000, 0.000, -0.677, 0.736))
        locations['B'] = Pose(Point(3.210, 3.511, 0.000), Quaternion(0.000, 0.000, 0.784, 0.620))
        locations['C'] = Pose(Point(-7.018, -0.943, 0.000), Quaternion(0.000, 0.000, -0.707,0.707))

        begin_location = Pose(Point(-3.0, 1, 0), Quaternion(0, 0, 0, 0))
        # 发布控制机器人的消息
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # 订阅move_base服务器的消息
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # 60s等待时间限制
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server")

        # 保存机器人在rviz中的初始位置
        initial_pose = PoseWithCovarianceStamped()

        # 确保有初始位置
        while initial_pose.header.stamp == "":
            rospy.sleep(1)

        rospy.loginfo("Starting navigation test")

        # 开始主循环、随机导航
        flag = 0
        for location in locations:

           # 设置目标点
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose = locations[location]
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()

           # 让用户知道下一个位置
            rospy.loginfo("Going to: " + str(location))

            # 向下一个位置进发
            self.move_base.send_goal(self.goal)

            # 五分钟时间限制
            finished_with_time = self.move_base.wait_for_result(rospy.Duration(120))

            # 查看是否成功到达
            if not finished_with_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    masktype = self.move_ok(location)
                    if (masktype == 'false'):
                        rospy.loginfo("next point")
                    else:
                        rospy.loginfo('find the person')
                        flag = 1
                else:
                    rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))
            if (flag == 1): break

        
        if (flag == 1):
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose = begin_location
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()

            rospy.loginfo("Going to: initial_pose")

            self.move_base.send_goal(self.goal)

            finished_with_time = self.move_base.wait_for_result(rospy.Duration(120))

            if not finished_with_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                else:
                    rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))
        

    def move_ok(self, pose_name):
        take_photo = takePhoto(pose_name)
        #保存之后得到照片
        imgPath = take_photo.take_a_photo()
        print(imgPath)

        print("开始进行口罩识别")
        return TrashCompare(imgPath)

    
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
    

if __name__ == '__main__':
    try:
        NavTest()
    except rospy.ROSInterruptException:
        rospy.loginfo("Random navigation finished.")
