#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy, cv2, cv_bridge, ast
from sensor_msgs.msg import Image

class takePhoto:
    def __init__(self, guest_name):
        self.name = guest_name
        #保存路径，需要替换成自己的
        self.savePath = '/home/pyj/catkin_workspace/src/myrobot/photo/' + self.name + '.jpg'
        self.bridge = cv_bridge.CvBridge()
        self.img = None

    def take_a_photo(self):
        #需要订阅cmera/rgb/image_raw这个话题
        #用opencv保存
        while self.img is None:
            try:
                self.img = rospy.wait_for_message('/camera/rgb/image_raw',
                                                  Image,
                                                  timeout=None)
                
            except:
                rospy.loginfo('failed')
                pass
        # Bridge and save the photo of the Trash
        self.image = self.bridge.imgmsg_to_cv2(self.img, desired_encoding='bgr8')
        cv2.imwrite(self.savePath, self.image[:,:])
        rospy.loginfo('%s\'s face photo has been saved in %s.' %
              (self.name, self.savePath))
        return self.savePath
    




def main():
    guest_name = 'Trash'
    take_photo = takePhoto(guest_name)
    take_photo.take_a_photo()


if __name__ == '__main__':
    main()
