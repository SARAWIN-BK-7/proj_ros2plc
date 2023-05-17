#!/usr/bin/env python3

# ROS 
import sys 
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import getch
import numpy as np 
import copy 

# from checkUR5pose import Ry, Rx

# import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
# from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg

# import actionlib
# from action_command.msg import *


# Constant Variable
# D2R = np.pi/180
# R2D = 180/np.pi


# Setup Moveit 
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

# Robot arm 
control_speed = 0.1
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

# Setup ROS Node 
# rospy.init_node('pick_and_place', anonymous=True)
# # pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
# rate = rospy.Rate(10) #10hz

# Save when use 
move_group.set_max_velocity_scaling_factor(control_speed)
move_group.set_max_acceleration_scaling_factor(control_speed)

move_group.set_goal_orientation_tolerance(0.02)
move_group.set_goal_position_tolerance(0.001)
move_group.set_planning_time(5)
move_group.set_num_planning_attempts(10)
move_group.allow_replanning(True)



display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
    )

# pose_goal = move_group.get_current_pose().pose

# class PickandPlace:
#     def __init__(self):
#         rospy.loginfo(":: Starting PickandPlace ::")

        
# pickplace = PickandPlace()

def main():
    rospy.init_node('pick_and_place', anonymous=True)
    # pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
    rate = rospy.Rate(10) #10hz

    while not rospy.is_shutdown():
        
        waypoints = []

        key = getch.getch().lower()
        print("please enter key :" , key)
        if key == 'p':
            
            pick_pose = move_group.get_current_pose().pose
            pick_pose = geometry_msgs.msg.Pose()
            # pick_pose.orientation.w = 1.0
            # pick_pose.position.x = 0.2
            # pick_pose.position.y = -0.2
            # pick_pose.position.z = 0.2

            pick_pose.orientation.w = 0.1
            pick_pose.position.x = -0.2
            pick_pose.position.y = 0.2
            pick_pose.position.z = 0.4
            move_group.set_pose_target(pick_pose)
            waypoints.append(copy.deepcopy(pick_pose))

            print('======================================================================================')
            print('waypoints', waypoints)
            (plan, fraction) = move_group.compute_cartesian_path(
                waypoints, 0.01, 0.0
            )
            print('plan', plan)
            print("===================================================================================== ")
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = robot.get_current_state()
            display_trajectory.trajectory.append(plan)

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