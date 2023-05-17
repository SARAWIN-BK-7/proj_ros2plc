#!/usr/bin/env python3

import rospy, sys, numpy as np
import moveit_commander
from copy import deepcopy
from geometry_msgs.msg import Twist
import moveit_msgs.msg
from sensor_msgs.msg import Image
from ur5_notebook.msg import Tracker2
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from time import sleep
tracker = Tracker2()

class ur5_mp_master:

    def __init__(self):
        rospy.init_node("motion_planning", anonymous=False)
        self.cxy_sub = rospy.Subscriber('cxy', Tracker2, self.tracking_callback, queue_size=1)
        self.cxy_pub = rospy.Publisher('cxy1', Tracker2, queue_size=1)
        self.phase = 1
        self.object_cnt = 0
        self.track_flag = False
        self.default_pose_flag = True
        self.cx = 400.0
        self.cy = 400.0
        self.points=[]
        self.state_change_time = rospy.Time.now()

        rospy.loginfo("Starting node moveit_cartesian_path")

        rospy.on_shutdown(self.cleanup)

        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        group_name = "manipulator"
        # Initialize the move group for the ur5_arm
        self.arm = moveit_commander.MoveGroupCommander(group_name)

        # Get the name of the end-effector link
        self.end_effector_link = self.arm.get_end_effector_link()

        # Set the reference frame for pose targets
        reference_frame = "world"

        # Set the ur5_arm reference frame accordingly
        self.arm.set_pose_reference_frame(reference_frame)

        # Allow replanning to increase the odds of a solution
        self.arm.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)
        self.arm.set_planning_time(0.1)
        self.arm.set_max_acceleration_scaling_factor(0.9999)
        self.arm.set_max_velocity_scaling_factor(0.9999)

        # Get the current pose so we can add it as a waypoint
        start_pose = self.arm.get_current_pose(self.end_effector_link).pose
        # print("start_pose", start_pose)

        # Initialize the waypoints list
        self.waypoints= []
        self.pointx = []
        self.pointy = []
        # Set the first waypoint to be the starting pose
        # Append the pose to the waypoints list
        wpose = deepcopy(start_pose)

        # Set the next waypoint to the right 0.5 meters

        wpose.position.x = 0.2
        wpose.position.y = -0.2
        wpose.position.z = 0.3
        self.waypoints.append(deepcopy(wpose))

        # wpose.position.x = 0.1052
        # wpose.position.y = -0.4271
        # wpose.position.z = 0.4005
        #
        # wpose.orientation.x = 0.4811
        # wpose.orientation.y = 0.5070
        # wpose.orientation.z = -0.5047
        # wpose.orientation.w = 0.5000

        # self.waypoints.append(deepcopy(wpose))


        if np.sqrt((wpose.position.x-start_pose.position.x)**2+(wpose.position.x-start_pose.position.x)**2 \
            +(wpose.position.x-start_pose.position.x)**2)<0.1:
            rospy.loginfo("Warnig: target position overlaps with the initial position!")

        # self.arm.set_pose_target(wpose)

        # Specify default (idle) joint states
        self.default_joint_states = self.arm.get_current_joint_values()
        self.default_joint_states[1] = -1.5667
        self.default_joint_states[0] = 3.14159 
        self.default_joint_states[2] = 1.89266
        self.default_joint_states[3] = -1.87721
        self.default_joint_states[4] = -1.5705
        self.default_joint_states[5] = 0.0

        self.arm.set_joint_value_target(self.default_joint_states)

        # Set the internal state to the current state
        self.arm.set_start_state_to_current_state()
        plan = self.arm.plan()

        self.arm.execute(plan)

        # Specify end states (drop object)
        #self.end_joint_states = deepcopy(self.default_joint_states)
        #self.end_joint_states[0] = -1.57691
        #self.end_joint_states[1] = -1.2166
        #self.end_joint_states[2] = 1.309
        # print ("End Joint", self.end_joint_states[2])

        #self.end_joint_states[1] = -1.3705

        self.transition_pose = deepcopy(self.default_joint_states)
        #self.transition_pose[0] = 1.57691
        #self.transition_pose[4] = -1.95
        # print ("Trans Joint", self.transition_pose[4])


    def cleanup(self):
        rospy.loginfo("Stopping the robot")

        # Stop any current arm movement
        self.arm.stop()

        #Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def tracking_callback(self, msg):

        self.track_flag = msg.flag1
        self.cx = msg.x
        self.cy = msg.y
        self.error_x = msg.error_x
        self.error_y = msg.error_y
        #print("SELF.ERROR_Y", self.error_y)
        if len(self.pointx)>9:
            self.track_flag = True
        if self.phase == 2:
            self.track_flag = False
            self.phase = 1


        if (self.track_flag and -0.6 < self.waypoints[0].position.x and self.waypoints[0].position.x < 0.6):
            self.default_pose_flag = False
            self.track_flag == True
            self.execute()

        else:
            if not self.default_pose_flag:
                self.track_flag = False
                self.execute()
                self.default_pose_flag = True



    def execute(self):
        if self.track_flag:


            # Get the current pose so we can add it as a waypoint
            start_pose = self.arm.get_current_pose(self.end_effector_link).pose

            # Initialize the waypoints list
            self.waypoints= []

            # Set the first waypoint to be the starting pose
            # Append the pose to the waypoints list
            wpose = deepcopy(start_pose)

            # wpose.position.x = -0.5215
            # wpose.position.y = 0.2014
            # wpose.position.z = 0.4102


            if len(self.pointx)>8:
                # print("SELF.POINTX > 8")
                if len(self.pointx)==9:
                    x_speed = np.mean(np.asarray(self.pointx[4:8]) - np.asarray(self.pointx[3:7]))
                    wpose.position.x += 3 * x_speed
                    wpose.position.z = 0.166
                    #wpose.position.x -= 0.03

                else:
                    if len(self.pointx)==11:
                        tracker.flag2 = 1
                        self.cxy_pub.publish(tracker)
                        


                    if len(self.pointx)<12:
                        x_speed = np.mean(np.asarray(self.pointx[4:8])-np.asarray(self.pointx[3:7]))
                        wpose.position.x += (x_speed-self.error_x*0.015/105)

                    else:
                        if tracker.flag2:
                            self.track_flag=False
                           
                        # transition_pose = deepcopy(start_pose)
                        # transition_pose.position.z = 0.4000
                        

                        # self.waypoints.append(deepcopy(transition_pose))

                        # self.arm.set_start_state_to_current_state()
                        # plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.02, 0.0, True)
                        rospy.sleep(5)
                        # self.arm.execute(plan)

                        # self.arm.set_max_acceleration_scaling_factor(.05)
                        # self.arm.set_max_velocity_scaling_factor(.05)



                        # self.arm.set_joint_value_target(self.transition_pose)
                        # self.arm.set_start_state_to_current_state()
                        # plan = self.arm.plan() 
                        # self.arm.execute(plan)

                        #print("Else where transition and end joint occurs")

                        # self.arm.set_joint_value_target(self.end_joint_states)
                        # self.arm.set_start_state_to_current_state()
                        # plan = self.arm.plan()
                        # self.arm.execute(plan)

                        if -0.1+0.02*self.object_cnt<0.2:

                            self.object_cnt += 1

                        self.waypoints = []
                        start_pose_now = self.arm.get_current_pose(self.end_effector_link).pose
                        transition_pose_now = deepcopy(start_pose_now)
                        transition_pose_now.position.x = -0.074933808768
                        transition_pose_now.position.y = -0.233472578779
                        transition_pose_now.position.z = 0.624066967802
                        transition_pose_now.orientation.x = 0.0143477952596
                        transition_pose_now.orientation.y = -0.00924712553566
                        transition_pose_now.orientation.z = 0.540255407415
                        transition_pose_now.orientation.w = 0.84132795401

                        #transition_pose.position.z = 0.2 + self.object_cnt*0.025
                        self.waypoints.append(deepcopy(transition_pose_now))

                        self.arm.set_start_state_to_current_state()
                        plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.02, 0.0, True)
                    
                        self.arm.execute(plan)

                        rospy.sleep(15) #for the second robot to come and catch the object
                        tracker.flag2 = 0
                        self.cxy_pub.publish(tracker)
                        rospy.sleep(10) #for the first robot to open the gripper fingers
                        self.phase = 2
                    

            # Set the next waypoint to the right 0.5 meters
            else:
                #wpose.position.x += 0.01
                wpose.position.x -= self.error_x*0.001/105
                wpose.position.y += self.error_y*0.001/105
                wpose.position.z = 0.224
                #wpose.position.z = 0.4005

            if self.phase == 1:

                self.waypoints.append(deepcopy(wpose))
                self.pointx.append(wpose.position.x)
                self.pointy.append(wpose.position.y)

                # Set the internal state to the current state
                # self.arm.set_pose_target(wpose)

                self.arm.set_start_state_to_current_state()

                # Plan the Cartesian path connecting the waypoints
                plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.01, 0.0, True)


                # plan = self.arm.plan()

                # If we have a complete plan, execute the trajectory
                if 1-fraction < 0.2:
                    rospy.loginfo("Path computed successfully. Moving the master.")
                    num_pts = len(plan.joint_trajectory.points)
                    rospy.loginfo("\n# intermediate waypoints= "+str(num_pts))
                    self.arm.execute(plan)
                    rospy.loginfo("Path execution complete.")
                else:
                    rospy.loginfo("Path planning failed")

        else:
            # Get the current pose so we can add it as a waypoint
            start_pose = self.arm.get_current_pose(self.end_effector_link).pose

            # Initialize the waypoints list
            self.waypoints= []
            self.pointx = []
            self.pointy = []
            # Set the first waypoint to be the starting pose
            # Append the pose to the waypoints list
            wpose = deepcopy(start_pose)

            # Set the next waypoint to the right 0.5 meters

            wpose.position.x = -0.466367197084
            wpose.position.y = -0.829194930697
            wpose.position.z =  0.304426967336

            wpose.orientation.x =  0.713999436472
            wpose.orientation.y =-9.88286656525e-05
            wpose.orientation.z = -0.700146244723
            wpose.orientation.w =  0.000175929092534
            self.pointx.append(wpose.position.x)
            self.pointy.append(wpose.position.y)
            self.waypoints.append(deepcopy(wpose))
            # Set the internal state to the current state
            # self.arm.set_pose_target(wpose)

            self.arm.set_start_state_to_current_state()

            # Plan the Cartesian path connecting the waypoints

            plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.01, 0.0, True)

            # plan = self.arm.plan()

            # If we have a complete plan, execute the trajectory
            if 1-fraction < 0.2:
                rospy.loginfo("Path computed successfully. Moving the arm.")
                num_pts = len(plan.joint_trajectory.points)
                rospy.loginfo("\n# intermediate waypoints = "+str(num_pts))
                self.arm.execute(plan)
                rospy.loginfo("Path execution complete.")
            else:
                rospy.loginfo("Path planning failed")


mp=ur5_mp_master()

rospy.spin()