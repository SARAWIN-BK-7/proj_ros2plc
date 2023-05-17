#!/usr/bin/env python3

# ROS 
import rospy
import rosnode
import tf
from tf.transformations import *

# Moveit
import moveit_commander
import moveit_msgs.msg

# Utility
from utils import *
import getch
import numpy as np
import time
import copy
import sys
from copy import deepcopy
import geometry_msgs.msg

# Image
import cv2, cv_bridge
from sensor_msgs.msg import Image

class CamTest:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",
                                         Image, self.image_callback)
    
    def image_callback(self,msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0 , 100 , 100])
        upper_red = np.array([10, 255, 225])
        mask = cv2.inRange(hsv, lower_red, upper_red)

        h,w,d = img.shape
        # print("print image size : ",h,w,d) # (480 640 3)

        masked = cv2.bitwise_and(img, img, mask=mask)
        cv2.namedWindow("window",1)
        cv2.imshow("window", masked)
        cv2.waitKey(5)

if __name__ == '__main__':
    rospy.init_node('Camtest',anonymous=True)
    cam = CamTest()
    rospy.spin()

