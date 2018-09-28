#!/usr/bin/env python
# A simple python program to move the franka robot back and forth. Only the pose of the end-effector is preserved.
# http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html

import rospy
import numpy as np
import tf
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
import sys

class Move():

    def __init__(self, *args):
        moveit_commander.roscpp_initialize(sys.argv)
        robot_state = moveit_commander.RobotCommander()
        robot_pose = moveit_commander.MoveGroupCommander("panda_arm")
        print  '#########  robot current pose ######', robot_pose.get_current_pose().pose

        scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("panda_arm")
        self. initial_pose = robot_pose.get_current_pose().pose
        # print self.current_pose.position.x, self.current_pose.orientation.x
        # self.group.set_goal_position_tolerance(0.01)
        # self.group.set_planner_id("FMTkConfigDefault")
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        print "============ Waiting for RVIZ..."
        rospy.sleep(1)


    def plan_execute(self, X, Y, Z, qx, qy, qz, qw):
        pose_target = PoseStamped()
        pose_target.header.frame_id = self.group.get_planning_frame()
        
        pose_target.pose.orientation.x = qx 
        pose_target.pose.orientation.y = qy
        pose_target.pose.orientation.z = qz
        pose_target.pose.orientation.w = qw

        pose_target.pose.position.x = X
        pose_target.pose.position.y = Y
        pose_target.pose.position.z = Z
       
        self.group.set_pose_target(pose_target)
        self.group.set_max_velocity_scaling_factor(0.15)     ## scaling down velocity
        # self.group.set_max_acceleration_scaling_factor     ## scaling down velocity
        plan = self.group.plan()
        print " Waiting while RVIZ displays plan..."
        rospy.sleep(0.5)

        while True:
            text = raw_input("============ Press Y to execute and N to terminate")
            if text == "Y" or text == "y":
                break
            if text == "N" or text == "n":
                self.group.clear_pose_targets()
                self.group.stop()
                raise ValueError('User wanted me to quit :(')
        print "Executing"
        self.group.execute(plan)

        return


if __name__ == "__main__":

    rospy.init_node('temp', anonymous=True)
    listener = tf.TransformListener()
    Run_Moveit = Move()
    initial_pose =  Run_Moveit.initial_pose
    while not rospy.is_shutdown():
        x = 0.35; y = 0.35; z = 0.8;   #  qx = -0.5; qy = .5; qz = 0; qw = np.sqrt((1 - qx** 2 - qy** 2 - qz** 2));
        qx = 1; qy = 0; qz = 0; qw = 0
        desired_pose = np.array([x, y, z, qx, qy, qz, qw])
        # Run_Moveit.plan_execute(x, y, z, qx, qy, qz, qw)
        # Run_Moveit.plan_execute(desired_pose[0], desired_pose[1], desired_pose[2], desired_pose[3], desired_pose[4], desired_pose[5], desired_pose[6])
        init_pose = np.array([initial_pose.position.x, initial_pose.position.y, initial_pose.position.z, initial_pose.orientation.x,
                                 initial_pose.orientation.y, initial_pose.orientation.z, initial_pose.orientation.w])
        while True:
            text = int(raw_input("============ Press 1: desired pose; 2: initial pose; 0: exit"))
            if text == 1:
                Run_Moveit.plan_execute(desired_pose[0], desired_pose[1], desired_pose[2], desired_pose[3],
                                        desired_pose[4], desired_pose[5], desired_pose[6])
                break
            if text ==2:
                Run_Moveit.plan_execute(init_pose[0], init_pose[1], init_pose[2], init_pose[3],
                                        init_pose[4], init_pose[5], init_pose[6])
            if text == 0:
                raise ValueError('User Quit')
            if text != 1 or text != 2 or text != 0:
                print 'Invalid choice'
                continue
        # while True:
        #     text = raw_input("============ Press Y to move the robot")
        #     if text == "Y" or text == "y":
        #         Run_Moveit.plan_execute(-x, y, z, qx, qy, qz, qw)
        #         break
        #     if text == "N" or text == "n":
        #         self.group.clear_pose_targets()
        #         self.group.stop()
        #         raise ValueError('User wanted me to quit :(')

    # except rospy.ROSInterruptException:
    #     print("program interupted before completion")
