#!/usr/bin/env python
from math import radians
import rospy

from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler


class InitialPose():
    def __init__(self):
        self.name = "initial_node"

        self.cmd_vel_pub = "/cmd_vel"
        self.initialpose_pub = "/initialpose"

        # set default pose
        self.default_pose = PoseWithCovarianceStamped()

        self.default_pose.pose.pose.position.x = -3.0
        self.default_pose.pose.pose.position.y = 1
        self.default_pose.pose.pose.position.z = 0.0
        self.default_pose.header.frame_id = "map"
        self.default_pose.pose.pose.orientation = Quaternion(
            *quaternion_from_euler(0, 0, 0))
        self.default_pose.pose.covariance = \
            [0.1, 0, 0, 0, 0, 0,
             0, 0.1, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0.1]

        # set default move_speed

        self.rotate = Twist()
        self.rotate.angular.z = radians(45)
        self.rotate.linear.x = 0

        self.stop = Twist()
        self.stop.linear.x = 0
        self.stop.angular.x = 0


    def run(self):
        self.init_node()
        rospy.loginfo("setting initial location")
        self.set_default_location()
        rospy.loginfo("starting rotating")
        self.self_rotate()

    def init_node(self):
        rospy.init_node(self.name, anonymous=False)

    def set_default_location(self):
        pub = rospy.Publisher(self.initialpose_pub, PoseWithCovarianceStamped, queue_size=1)

        for _ in range(0, 5):
            rospy.sleep(1)
            pub.publish(self.default_pose)

    def self_rotate(self):
        pub = rospy.Publisher(self.cmd_vel_pub, Twist, queue_size=10)
        r = rospy.Rate(10)

        for _ in range(100):
            pub.publish(self.rotate)
            r.sleep()
    
        pub.publish(self.stop)



if __name__ == "__main__":

    app = InitialPose()
    app.run()    