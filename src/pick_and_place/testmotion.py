#!/usr/bin/env python3

# Ros 
import rospy 
import rosnode
import tf 
from tf.transformations import *
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg

# Message and Convert 
import geometry_msgs.msg 

# Moveit 
import moveit_commander 
import moveit_msgs.msg 

# Utility 
import sys 
import numpy as np
import time 
import getch 
import copy 

# Image processing 
import cv2 as cv 
import cv_bridge 


# Setup moveit 
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()


rospy.init_node("Motion_Robot",anonymous=True)
pub = rospy.Publisher('Robottiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
rate = rospy.Rate(10) # 10 Hz

control_speed = 1.0 # Simulation 
group_name = "manipulator"

move_group = moveit_commander.MoveGroupCommander(group_name)
move_group.set_max_velocity_scaling_factor(control_speed)
move_group.set_max_acceleration_scaling_factor(control_speed)
move_group.set_goal_orientation_tolerance(np.deg2rad(5))
move_group.set_goal_position_tolerance (0.001)
move_group.set_planning_time(5)
move_group.set_num_planning_attempts(10)


display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_plnaned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
) 

def main(): 
    while not rospy.is_shutdown():
        
        waypoints = []
        
        key = getch().lower()
        print("key :", key)
        
        if key == 'o':
            wpose = move_group.get_current_pose().pose 
            wpose.position.x = 0.324920
            wpose.position.y = 0.423205
            wpose.position.z = 0.754333
            
            wpose.orientation.x = 0.14565
            wpose.orientation.y = 0.2
            wpose.orientation.z = 0.75
            wpose.orientation.w = -0.2
            move_group.set_pose_traget(wpose)
            waypoints.append(copy.deepcopy(wpose))

            print("========================================================================")
            print("waypoints : ",waypoints)
            (plan, fraction) = move_group.compute_cartesian_path(
                waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
            )  # jump_threshold

            print("plan :",plan)
            print("========================================================================")
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = robot.get_current_state()
            display_trajectory.trajectory.append(plan)
                # Publish
            display_trajectory_publisher.publish(display_trajectory)

            move_group.execute(plan, wait=True)
            
        if key == 'q':
            break

        rate.sleep()            

if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException
        pass 