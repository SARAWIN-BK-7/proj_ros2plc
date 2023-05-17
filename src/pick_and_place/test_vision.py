#!/usr/bin/python3

#ROS
import rospy 

# Message 
from ur5_notebook.msg import Tracker2

# Image 
import cv2, cv_bridge
from sensor_msgs.msg import Image

#Utility 
import numpy as np
import sys 

tracker = Tracker2() 


class UR5_Vision:
    def __init__(self):
        rospy.init_node("ur5_vision",anonymous=True)
        self.track_flag = False
        self.default_pose_flag = True
        self.cx = 400.0
        self.cy = 400.0
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('usb_cam/image_raw', Image, self.image_callback)
        self.cxy_pub = rospy.Publisher('cxy', Tracker2 ,queue_size=1)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # END HSV
        # BEGIN FILTER
        lower_red = np.array([ 0,  100, 100])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        (_,cnts, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #area = cv2.contourArea(cnts)
        h, w, d = image.shape
        # print h, w, d  (800,800,3)
        #BEGIN FINDER
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
     
     
     
            for i, c in enumerate(cnts):
                area = cv2.contourArea(c)
                if area > 7500:
                    self.track_flag = True
                    self.cx = cx
                    self.cy = cy
                    self.error_x  =self.cx - w/2
                    self.error_y = self.cy - (h/2+195)
                    tracker.x = cx 
                    tracker.y = cy 
                    tracker.flag1 = self.track_flag
                    tracker.error_x = self.error_x
                    tracker.error_y = self.error_y
                    
                    cv2.circle(image, (cx, cy), 10, (0, 0, 0), -1)
                    cv2.putText(image, "({}, {})".format(int(cx), int(cy)), (int(cx-5), int(cy+15)), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv2.drawContours(image, cnts, -1, (255, 255, 255), 1)
                    
                    break
                else:
                    self.track_flag = False
                    tracker.flag1 = self.track_flag
                    tracker.x = cx
                    tracker.y = cy
                    tracker.error_x = 0
                    tracker.error_y = 0
                    
                    
        self.cxy_pub.publish(tracker)
        # self.masked = cv2.bitwise_and(image, image, mask=mask)
        cv2.namedWindow("window", 1)
        cv2.imshow("window", image)
        cv2.waitKey(5)
        
follower=UR5_Vision()
rospy.spin()