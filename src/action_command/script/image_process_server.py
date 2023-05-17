#! /usr/bin/env python

# Ros
import rospy
import actionlib

# Utility
import numpy as np
import copy

# msg & convert
from action_command.msg import *
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray,UInt64MultiArray
# Image
import cv2

bridge=CvBridge()

def resize(img,scale_percent):
    # scale_percent = 50 # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    
        # resize image
    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    return resized

class image_processServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('image_process', image_processAction, self.rgb_callback, False)
    self.feedback = image_processFeedback()
    self.result = image_processResult()
    self.rgb_image = None
    self.server.start()

  def rgb_callback(self, received_image):
    try:
      self.rgb_image = bridge.imgmsg_to_cv2(received_image.image_input, "bgr8")
      
      while True:
        cv2.imshow("server Image",resize(self.rgb_image,50))
        if cv2.waitKey(1) & 0xFF==ord('q'):
            cv2.destroyAllWindows()
            break
      
      cv2.putText(self.rgb_image, f'Pic Action', (800,500),cv2.FONT_HERSHEY_SIMPLEX,3,(0,0,0),10)
      cv2.putText(self.rgb_image, f'Pic Action', (800,500),cv2.FONT_HERSHEY_SIMPLEX,3,(255,255,150),9)

      self.result.image_result =  bridge.cv2_to_imgmsg(self.rgb_image, "bgr8") 
      floatarray = Float64MultiArray()
      floatarray.data = [1.2, 2.3, 3.4]
      uintarray = UInt64MultiArray()
      uintarray.data = [1, 2, 3]
      self.result.Float64_array = floatarray
      self.result.Uint64_array = uintarray
      self.server.set_succeeded(self.result)

    except CvBridgeError as e:
      print(e)
    


  # def execute(self, image_input):
  #   # Do lots of awesome groundbreaking robot stuff here
  #   print(f" goal.pose : {image_input.pose}")
  #   # self.feedback.result.data = "6666"
  #   # self.server.publish_feedback(self.feedback)
  #   self.result.task_result.data = "complete"
  #   self.server.set_succeeded(self.result)


if __name__ == '__main__':
  rospy.init_node('arm_tf_msg_server')
  server = image_processServer()
  rospy.spin()