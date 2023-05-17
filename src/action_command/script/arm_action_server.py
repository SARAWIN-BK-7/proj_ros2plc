#! /usr/bin/env python

import rospy
import actionlib

from action_command.msg import *

class GoplaceServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('go_place', Go_placeAction, self.execute, False)
    self.feedback = Go_placeFeedback()
    self.result = Go_placeResult()
    
    self.server.start()

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    print(f" goal.command : {goal.pos_num}")
    print(f" goal.release_type : {goal.release_type}")
    # self.feedback.result.data = "6666"
    # self.server.publish_feedback(self.feedback)
    self.result.task_result.data = "complete"
    self.server.set_succeeded(self.result)


if __name__ == '__main__':
  rospy.init_node('go_place_server')
  server = GoplaceServer()
  rospy.spin()