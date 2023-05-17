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

if __name__ == '__main__':
  rospy.init_node('image_process_client')
  client = actionlib.SimpleActionClient('image_process', image_processAction)
  client.wait_for_server()

  color_image = cv2.imread(f'/home/oongking/config_ws/src/action_command/data/test_img.png')

  image_action_input = image_processGoal()
  image_action_input.image_input = bridge.cv2_to_imgmsg(color_image, "bgr8")

  client.send_goal(image_action_input)

  client.wait_for_result(rospy.Duration.from_sec(5.0))
  process_result = client.get_result()
  out_img =  bridge.imgmsg_to_cv2(process_result.image_result, "bgr8")
  floatarray = np.asarray(process_result.Float64_array.data)
  intarray = np.asarray(process_result.Uint64_array.data)

  while True:
    cv2.imshow("client Image",resize(out_img,50))
    if cv2.waitKey(1) & 0xFF==ord('q'):
      cv2.destroyAllWindows()
      break
  
  print(f"floatarray : {floatarray}")
  print(f"intarray : {intarray}")

  # print(f"get result : {client.get_result()}")