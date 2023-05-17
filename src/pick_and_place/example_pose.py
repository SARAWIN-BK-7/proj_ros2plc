#!/usr/bin/env python3

# ROS
import sys
import copy
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random
import time
import getch
import numpy as np

ROBOT_NAME = "ur5"
if ROBOT_NAME == "ur5":
    GROUP_NAME_ARM = "manipulator"
    GROUP_NAME_GRIPPER = "gripper"

    GROIPPER_FRAME = "ee_link"

    FIXED_FRAME = "base_link"

    GRIPPER_CLOSED = 0.8
    GRIPPER_OPEN = 0.1

    GRIPPER_JOINT_NAME = ['robotiq_85_left_knuckle_joint']

    GRPPER_EFFORT = [0.1]

class TestPick():
    def __init__(self):
        
        roscpp_initialize(sys.argv)
        rospy.init_node('moveit_py_demo', anonymous=True)

        scene = PlanningSceneInterface()
        robot = RobotCommander()

        arm = MoveGroupCommander(GROUP_NAME_ARM)
        gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)

        arm.set_goal_orientation_tolerance(0.005)
        arm.set_planning_time(5)
        arm.set_num_planning_attempts(5)
        eef = arm.get_end_effector_link()

        rospy.sleep(2)
        arm.allow_replanning(True)

        arm.set_named_target("up")
        arm.go(wait=True)
        rospy.sleep(3)

        print("gripper open")
        gripper.set_named_target("open")
        gripper.go(wait=True)

        rospy.sleep(1)

        print('going to pick up pose')
        rospy.sleep(5)

        gripper.set_named_target("close")
        gripper.go(wait=True)
        rospy.sleep(1)

if __name__ == "__main__":
    try:
        TestPick()
    except:
        rospy.ROSInterruptException
        pass