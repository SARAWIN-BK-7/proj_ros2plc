#! /usr/bin/env python

# import roslib
# roslib.load_manifest('my_pkg_name')
import rospy
import actionlib
from action_command.msg import *

if __name__ == '__main__':
    rospy.init_node('do_dishes_client')
    client = actionlib.SimpleActionClient('go_place', Go_placeAction)
    client.wait_for_server()

    goal = Go_placeGoal()
    # Fill in the goal here
    goal.pos_num = 5
    goal.release_type.data = "opennnnnnn"
    client.send_goal(goal)

    client.wait_for_result(rospy.Duration.from_sec(5.0))

    print(f"get result : {client.get_result()}")