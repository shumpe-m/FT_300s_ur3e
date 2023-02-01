#!/usr/bin/env python
# Python 2 compatibility imports
"""
Copyright (c) 2008-2013, Willow Garage, Inc.
Copyright (c) 2015-2019, PickNik, LLC.
https://github.com/ros-planning/moveit_tutorials/blob/master/LICENSE.txt
"""
from __future__ import print_function
from six.moves import input

import sys
import time
import copy
import tf
import actionlib
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from gazebo_msgs.msg import ModelStates

import numpy as np

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt
    tau = 2.0 * pi
    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


"""Modifications copyright (C) 2022 Shumpe MORITA."""
def all_close(goal, actual, tolerance, premitive = ""):
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)
    
    elif premitive == "ori":
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        #print(d, tolerance,cos_phi_half,cos(tolerance / 2.0))
        #print(cos_phi_half, cos(tolerance / 2.0))
        return cos_phi_half >= cos(tolerance / 2.0)


    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        print(d, tolerance,cos_phi_half,cos(tolerance / 2.0))
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class arm_control(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self, name = 'arm'):
        super(arm_control, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("ur_planner", anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        self.name = name
        group_name = self.name
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        move_group.set_end_effector_link("ur_gripper_tip_link")
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        self.max_x = 0.2
        self.min_x = -0.21
        self.max_y = 0.45
        self.min_y = 0.25
        self.max_z = 0.2
        self.min_z = 0.01 + 0.02 # 0.02=offset


    def go_to_joint_state(self, joint_ang):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal = joint_ang

        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_position(self, position=[0,0,0]):
        move_group = self.move_group
        move_group.set_end_effector_link("ur_gripper_tip_link")

        wpose = move_group.get_current_pose().pose
        current_pose = copy.deepcopy(wpose)
        # Position set
        wpose.position.x = position[0]
        wpose.position.y = position[1]
        wpose.position.z = position[2]
        move_group.set_pose_target(wpose)
        # Founding motion plan
        plan = move_group.plan()
        if plan.joint_trajectory.header.frame_id == "": # No motion plan found.
            print("No motion plan found. position = ", position)
        # Execute plan
        move_group.execute(plan, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        return plan.joint_trajectory.header.frame_id!=[], wpose


    def go_to_pose(self, pose, ori = [0.0, 0.0, 0.0, 0.0]):
        move_group = self.move_group
        move_group.set_end_effector_link("ur_gripper_tip_link")
        wpose = move_group.get_current_pose().pose
        if type(pose) == geometry_msgs.msg.Pose:
            wpose.position = pose.position
        elif type(pose) == list:
            wpose.position.x = pose[0]
            wpose.position.y = pose[1]
            wpose.position.z = pose[2]
        else:
            print("The type of variables is different: pose")


        if type(ori) == geometry_msgs.msg._Quaternion.Quaternion:
            wpose.orientation = ori
        elif type(ori) == list:
            wpose.orientation.x = ori[0]
            wpose.orientation.y = ori[1]
            wpose.orientation.z = ori[2]
            wpose.orientation.w = ori[3]
        else:
            print("The type of variables is different: ori")

        move_group.set_pose_target(wpose)

        plan = move_group.plan()
        # if plan.joint_trajectory.header.frame_id == "": # No motion plan found.
        #     print("No motion plan found. position = ", pose, q)
        move_group.execute(plan, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        return plan.joint_trajectory.points!=[]

    def rot_motion(self, pose, ori = [1.0, 0.0, 0.0, 0.0]):
        move_group = self.move_group
        move_group.set_end_effector_link("ur_gripper_tip_link")
        wpose = move_group.get_current_pose().pose
        wpose.position = pose.position
        current_pose = copy.deepcopy(wpose)

        if type(ori) == geometry_msgs.msg._Quaternion.Quaternion:
            wpose.orientation = ori
        elif type(ori) == list:
            wpose.orientation.x = ori[0]
            wpose.orientation.y = ori[1]
            wpose.orientation.z = ori[2]
            wpose.orientation.w = ori[3]
        else:
            print("The type of variables is different.")

        move_group.set_pose_target(wpose)

        print("plan")
        plan = move_group.plan()
        print("exwcute")
        move_group.execute(plan, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        return plan.joint_trajectory.points!=[]


    def plan_path(self, scale=0.1):
        move_group = self.move_group
        waypoints = []
        joint_goal = move_group.get_current_joint_values()
        wpose = move_group.get_current_pose().pose
        wpose.position.y = 0.2
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.02, 0.0  # waypoints to follow  # eef_step
        )
        return plan, fraction


    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)

    def setup_bias(self):
        pose = self.print_current_pose()
        self.x_bias = pose.position.x

        self.z_bias = 0.88

    def execute_plan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)

    def euler_to_quaternion(self, euler = [3.140876586229683, 0.0008580159308600959, -0.0009655065200909574]):
        # Convert Euler Angles to Quaternion

        if type(euler) == geometry_msgs.msg.Pose:
            print("The type of variables is different.")
        elif type(euler) == list:
            q = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])
        else:
            print("The type of variables is different.")

        return q

    def quaternion_to_euler(self, quaternion):
        # Convert Quaternion to Euler Angles
        
        if type(quaternion) == geometry_msgs.msg._Quaternion.Quaternion:
            e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        elif type(quaternion) == list:
            e = tf.transformations.euler_from_quaternion((quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
        else:
            print("The type of variables is different.")

        return e

    def print_current_pose(self, end_effector_link_name = "ur_gripper_tip_link"):
        move_group = self.move_group
        move_group.set_end_effector_link(end_effector_link_name)
        wpose = move_group.get_current_pose().pose
        return wpose

    def to_list(self, pose):
        x, y, z, _, _, _, _ = pose_to_list(pose)
        data = np.array([[x,y,z]])
        return data

    def save_array(self, data_name, data, height, r_scale):
        file_name = data_name + "_height_" + str(height) + "_rscale_" + str(r_scale) 
        np.save(file_name, data, fix_imports=True)


def main():
    try:
        motion = arm_control("arm")
        #joint_ang = [1.57, 0, -0.03, -1.57, -1.57, 0]
        #joint_ang = [1.57, -1.57/2, 1.57/2, -1.57, -1.57, 0]

        joint_ang = [1.57, -1.57/2, 1.57/2, -1.57, -1.57, 0] #0~3.14, -3.14~0, 
        goal_data = np.array([])
        actual_data = np.array([])
        success_data = np.array([])
        # motion.setup_bias()
        motion.go_to_joint_state(joint_ang)
        default_pose = motion.print_current_pose()

        success, goal_pose = motion.go_to_position(position = [-0.21, 0.35, 0.03])

    
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()