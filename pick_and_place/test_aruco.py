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
import os 

# Image
import cv2, cv_bridge
from sensor_msgs.msg import Image

AruCo_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
AruCo_params = cv2.aruco.DetectorParameters_create()
board = cv2.aruco.GridBoard_create(5, 7, 0.024, 0.01, AruCo_dict)

cam = "usbcam"

K = np.array([[619.5831171360619, 0, 310.7132865814095],
                                       [0, 622.0241548644699, 266.682630120719],
                                       [0, 0, 1]])
dist = np.array([[0.115261367003599, -0.201086925594918, -0.001387151733832904, 0.001893161766540489, 0]])

def draw_box(img, corners, imgpts):
    imgpts = np.int32(imgpts).reshape(-1,2)
        # draw ground floor in green
    img = cv2.drawContours(img, [imgpts[:4]],-1,(0,255,0),-3)
        # draw pillars in blue color
    for i,j in zip(range(4),range(4,8)):
        img = cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]),(255),10)
         # draw top layer in red color
    img = cv2.drawContours(img, [imgpts[4:]],-1,(0,0,255),10)
    return img
def augmented_image(frame,im_src, pts_src, pts_dst):

    # Calculate Homography
    h, status = cv2.findHomography(pts_src, pts_dst)
    # Warp source image to destination based on homography
    warped_image = cv2.warpPerspective(im_src, h, (frame.shape[1],frame.shape[0]))
    # Prepare a mask representing region to copy from the warped image into the original frame.
    mask = np.zeros([frame.shape[0], frame.shape[1]], dtype=np.uint8)
    cv2.fillConvexPoly(mask, np.int32(pts_dst), (255, 255, 255), cv2.LINE_AA)
    im_out = cv2.add(frame, warped_image, mask=cv2.bitwise_not(mask))
    im_out = cv2.add(im_out, warped_image)

    return im_out

def write_text(img, pose, dy, text) :
    x0 = pose[0]
    y0 = pose[1]
    for i, line in enumerate(text.split('\n')) :
        y = y0 + i*dy
        cv2.putText(img, line, np.int32([x0, y]), cv2.FONT_HERSHEY_COMPLEX, 0.45, (25,25,25), 2)
   
class CamTest:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",
                                         Image, self.image_callback)
        
######################## draw AruCo frame ID #########################
    # def image_callback(self, msg):
    #     image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    #     # converr bgr to gray
    #     gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    #     # detected ArUco marker
    #     corners, ids, _ = cv2.aruco.detectMarkers(gray, AruCo_dict, parameters=AruCo_params)

    #     # draw marker
    #     if ids is not None:
    #         cv2.aruco.drawDetectedMarkers(image, corners, ids)

    #         # pinpoint marker
    #         for i in range(len(ids)):
    #             rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.05, K, dist)
    #             # ทำสิ่งที่คุณต้องการกับตำแหน่งของ marker ที่ได้ (rvec, tvec)
    #             # cv2.aruco.drawFrameAxes(image, K, dist, rvec, tvec, 0.05)
    #     cv2.imshow("Window", image)
    #     cv2.waitKey(1)
###################################################################

# drawFrame Axis , id, pose(x, y, z)

    def image_callback(self, msg): 
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print("Error", e)
            return 
        
        img = cv_image.copy()
        markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(img, AruCo_dict, parameters=AruCo_params)
        if len(markerCorners) > 0:
           img = cv2.aruco.drawDetectedMarkers(img, markerCorners)

           rvecs, tvecs, points = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 0.05, K, dist)

           for (rvec, tvec, id, corner) in zip(rvecs, tvecs, markerIds, markerCorners):
               img = cv2.drawFrameAxes(img, K, dist, rvec, tvec, 0.05)
               x = tvec[0, 0]
               y = tvec[0, 1]
               z = tvec[0, 2]
               text = "id: {}\n pose:\n {:.3f}\n {:.3f}\n {:.3f}".format(id, x, y, z)
               cX = (corner[0, 0][0] + corner[0, 2][0]) / 2
               cY = (corner[0, 0][1] + corner[0, 2][1]) / 2
               write_text(img, (cX, cY), 20, text)
           ret, brvec, btvec = cv2.aruco.estimatePoseBoard(markerCorners, markerIds, board, K, dist, rvecs, tvecs)
           if ret:
               img = cv2.drawFrameAxes(img, K, dist, brvec, btvec, 0.05)

        cv2.imshow('Image', img)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('Camtest',anonymous=True)
    cam = CamTest()
    rospy.spin()

