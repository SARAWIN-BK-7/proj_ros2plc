#! /usr/bin/env python

import rospy
import actionlib

from action_command.msg import *

class arm_tf_msgServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('arm_tf_msg', arm_tf_msgAction, self.execute, False)
    self.feedback = arm_tf_msgFeedback()
    self.result = arm_tf_msgResult()
    
    self.server.start()

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    print(f" goal.pose : {goal.pose}")
    # self.feedback.result.data = "6666"
    # self.server.publish_feedback(self.feedback)
    self.result.task_result.data = "complete"
    self.server.set_succeeded(self.result)


if __name__ == '__main__':
  rospy.init_node('arm_tf_msg_server')
  server = arm_tf_msgServer()
  rospy.spin()