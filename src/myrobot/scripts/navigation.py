#!/usr/bin/env python
#-*- coding:utf-8   -*-

'''
test sample
"{position: {x: 1.536250, y: 0.580879, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
'''

from pickle import load
import rospy
import actionlib

from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler

# 引入照相
from take_a_photo import takePhoto
from object_detect import *

class NavModule():
    def __init__(self):
        rospy.init_node("nav_wrapper", anonymous=True)
        rospy.on_shutdown(self.shutdown)
        self.goal = MoveBaseGoal()
        self.have_goal = False
        self.move_action_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.masktype = "unknown"
        self.move_action_client.wait_for_server(rospy.Duration(60))

        # 保存机器人在rviz中的初始位置
        initial_pose = PoseWithCovarianceStamped()

        # 确保有初始位置
        while initial_pose.header.stamp == "":
            rospy.sleep(1)

        rospy.loginfo("Starting navigation")       

    def set_goal(self, pose, pose_name): 
        '''
        设置目标
        传入参数:
        - pose: geometry_msgs.msg.Pose
        功能:
        设置导航的目标
        '''
        rospy.loginfo("get a goal, x: {}, y:{}, z:{}".format(pose.position.x, pose.position.y, pose.position.z))
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose = pose
        self.pose_name = pose_name
        self.have_goal = True

    def move(self, secs=120): #移动到目标点
        

        if not self.have_goal:
            rospy.logerr("please set a goal first")
            return 
        
        rospy.loginfo("starting moving")
        self.have_goal = False

        self.move_action_client.send_goal(self.goal)
        finish_res = self.move_action_client.wait_for_result(rospy.Duration(secs))

        state = ""
        if not finish_res:
            self.move_action_client.cancel_goal()
            rospy.loginfo("not finished")
        else:
            code = self.move_action_client.get_state()
            state = code
            if code == GoalStatus.SUCCEEDED:
                rospy.loginfo("goal succeed")
            else:
                rospy.loginfo("something is wrong, code is {}".format(code))
        return state

    def object_detect(self): # 在这里加入到达目标之后的逻辑
        #TODO
        # 调用take_photo代码
        take_photo = takePhoto(self.pose_name)
        #保存之后得到照片
        imgPath = take_photo.take_a_photo()
        rospy.loginfo("开始进行口罩识别")
        self.masktype = TrashCompare(imgPath)
        return self.masktype
        rospy.loginfo(self.masktype)
        # pass

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_action_client.cancel_goal()

if __name__ == "__main__":
    app = NavModule()
    locations = dict()

    locations['A'] = Pose(Point(5.246, -4.153, 0.000), Quaternion(0.000, 0.000, -0.677, 0.736))
    locations['B'] = Pose(Point(3.210, 3.511, 0.000), Quaternion(0.000, 0.000, 0.784, 0.620))
    locations['C'] = Pose(Point(-7.018, -0.943, 0.000), Quaternion(0.000, 0.000, -0.707,0.707))

    initial_pose = PoseWithCovarianceStamped()

    initial_pose.pose.pose.position.x = -3.0
    initial_pose.pose.pose.position.y = 1
    initial_pose.pose.pose.position.z = 0.0
    initial_pose.header.frame_id = "map"
    initial_pose.pose.pose.orientation = Quaternion(
            *quaternion_from_euler(0, 0, 0))
    initial_pose.pose.covariance = \
            [0.1, 0, 0, 0, 0, 0,
             0, 0.1, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0.1]

    flag = 0
    for location in locations:
        rospy.loginfo("Going to: " + str(location))
        app.set_goal(locations[location], location)
        state = app.move()
        if state != GoalStatus.SUCCEEDED: continue
        masktype = app.object_detect()
        if (masktype == 'unknown'):
            rospy.loginfo("not finished")
        elif (masktype == 'false'):
            rospy.loginfo('next point') 
        else:
            rospy.loginfo('find the person')
            rospy.loginfo("Going to: initial_pose")
            app.set_goal(initial_pose.pose.pose, "initial_pose")
            app.move() 
            flag = 1
            break
    if (flag == 0):
        rospy.loginfo('Not find the Person')
    