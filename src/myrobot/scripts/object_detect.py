#!/usr/bin/env python
# -*- coding: utf-8 -*-

#导入srv
import rospy
import numpy as np
import cv2
import matplotlib.pyplot as plt



def TrashCompare(imgPath):

    base_path="/home/pyj/catkin_workspace/src/myrobot/photo/kouzhao_tamplate.png"
    

    # print(base_path+tamplate_pic_path[i])
    img1 = cv2.imread(imgPath,0)            # 传入client发送过来的路径
    img2 = cv2.imread(base_path,0) # trainImage

    # Initiate ORB detector
    orb = cv2.ORB_create()
    # find the keypoints and descriptors with ORB
    kp1, des1 = orb.detectAndCompute(img1,None)
    kp2, des2 = orb.detectAndCompute(img2,None)
    # create BFMatcher object
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    # Match descriptors.
    matches = bf.match(des1,des2)
    # Sort them in the order of their distance.
    matches = sorted(matches, key = lambda x:x.distance)
  
    mask_type = "false"
    if len(matches)>50:
        rospy.loginfo("success")
        mask_type="success"
    else: rospy.loginfo("false")
   # Draw first 10 matches.
   # res.append(len(matches))
    #img3 = cv2.drawMatches(img1,kp1,img2,kp2,matches[:30],None, flags=2)
    #plt.imshow(img3),plt.show()
    return mask_type

   
if __name__ == '__main__':
    TrashCompare('/home/pyj/catkin_workspace/src/myrobot/photo/1.jpg')