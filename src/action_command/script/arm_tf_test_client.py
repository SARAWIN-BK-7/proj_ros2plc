#! /usr/bin/env python

# import roslib
# roslib.load_manifest('my_pkg_name')
import rospy
import actionlib
from action_command.msg import *

if __name__ == '__main__':
    rospy.init_node('arm_tf_msg_client')
    client = actionlib.SimpleActionClient('arm_tf_msg', arm_tf_msgAction)
    client.wait_for_server()

    goal = arm_tf_msgGoal()
    # Fill in the goal here
    goal.pose.position.x = 0
    goal.pose.position.y = 0
    goal.pose.position.z = 0
    goal.pose.orientation.x = 0
    goal.pose.orientation.y = 0
    goal.pose.orientation.z = 0
    goal.pose.orientation.w = 1

    client.send_goal(goal)

    client.wait_for_result(rospy.Duration.from_sec(5.0))

    print(f"get result : {client.get_result()}")