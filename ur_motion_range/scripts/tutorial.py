#!/usr/bin/env python
# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import time
import copy
import tf
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from gazebo_msgs.msg import ModelStates

import numpy as np
import math

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py
def all_close(goal, actual, tolerance, premitive = ""):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
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


class ur_control(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self, name = 'manipulator'):
        super(ur_control, self).__init__()

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
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        # ros message
        self.sub_vector = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callbackVector)
        self.model_state = ModelStates()

        self.x_bias = 0.11
        self.z_bias = 0.88

    def callbackVector(self, msg):
        self.model_state = msg

    def go_to_joint_state(self, joint_ang):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        #for idx range(len(joint_ang)):
        #    joint_goal[idx] = joint_ang[idx]
        joint_goal = joint_ang

        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, scale = 0.1, r = 0.54, theta = 0, height = 0):
        move_group = self.move_group
        move_group.set_end_effector_link("tool0")
        wpose = move_group.get_current_pose().pose
        current_pose = copy.deepcopy(wpose)
        
        wpose.position.x = r * np.cos(theta + np.pi/2) + self.x_bias * np.sin(theta + np.pi/2)
        wpose.position.y = r * np.sin(theta + np.pi/2) + self.x_bias * np.cos(theta + np.pi/2)
        wpose.position.z = height + self.z_bias
        
        move_group.set_pose_target(wpose)
        plan = move_group.plan()
        move_group.execute(plan, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        return plan.joint_trajectory.points!=[], wpose


    def rot_motion(self, pose, q = [0.0, 0.0, 0.0, 0.0]):
        move_group = self.move_group
        move_group.set_end_effector_link("ur_gripper_tip_link")
        wpose = move_group.get_current_pose().pose
        wpose.position = pose
        current_pose = copy.deepcopy(wpose)
        # wpose.orientation.x = current_pose.orientation.x + q[0]
        # wpose.orientation.y = current_pose.orientation.y + q[1]
        # wpose.orientation.z = current_pose.orientation.z + q[2]
        # wpose.orientation.w = current_pose.orientation.w + q[3]
        wpose.orientation.x = q[0]
        wpose.orientation.y = q[1]
        wpose.orientation.z = q[2]
        wpose.orientation.w = q[3]

        move_group.set_pose_target(wpose)
        plan = move_group.plan()
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
        pose = print_current_pose()
        self.x_bias = pose.position.x

        self.z_bias = 0.88

    def execute_plan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)

    def euler_to_quaternion(self, euler = [3.140876586229683, 0.0008580159308600959, -0.0009655065200909574]):
        """Convert Euler Angles to Quaternion
        """
        q = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])
        return q

    def quaternion_to_euler(self, quaternion):
        """Convert Quaternion to Euler Angles
        """
        e = tf.transformations.euler_from_quaternion((quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
        return e

    def end_effector_set(self):
        move_group = self.move_group
        move_group.set_end_effector_link("ur_gripper_tip_link")

    def print_current_pose(self):
        move_group = self.move_group
        wpose = move_group.get_current_pose().pose
        spawn_pose_x = 0.11
        spawn_pose_z = 0.87
        #wpose.position.x += spawn_pose_x
        #wpose.position.z += spawn_pose_z
        #e = self.quaternion_to_euler([wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w])
        #print(wpose)
        #print("euler:", e)
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
        print("----------------------------------------------------------")
        motion = ur_control("arm")
        #joint_ang = [1.57, 0, -0.03, -1.57, -1.57, 0]
        #joint_ang = [1.57, -1.57/2, 1.57/2, -1.57, -1.57, 0]

        dis = 0.54
        r_slope = dis - 0.20
        joint_ang = [1.57, -1.57/2, 1.57/2, -1.57, -1.57, 0] #0~3.14, -3.14~0, 
        default_ori = [3.140876586229683, 0.0008580159308600959, -0.0009655065200909574]
        goal_data = np.array([])
        actual_data = np.array([])
        success_data = np.array([])
        motion.setup_bias()

        ite1 = 5
        ite2 = 6
        height = 0.0
        r_scale = 0.0 # max = 1, min = 0

        # for h_dx in range(6):
        #     for r_dx in range(6):
        #         for theta1_dx in range(ite1):
        #                 motion.go_to_joint_state(joint_ang)
        #                 #print(motion.print_current_pose())
        #                 success, goal_pose = motion.go_to_pose_goal(r = dis - r_slope * r_scale, theta = np.pi/2 * theta1_dx/4, height = height)
        #                 print("############### first motion:", success, ", ", theta1_dx + 1, "/", ite1)
        #                 goal_data = motion.to_list(goal_pose) if goal_data.shape == (0,) else np.concatenate([goal_data, motion.to_list(goal_pose)], 0)
        #                 if success:
        #                     pose = motion.print_current_pose()
        #                     actual_data = motion.to_list(pose) if actual_data.shape == (0,) else np.concatenate([actual_data, motion.to_list(pose)], 0)
        #                     for theta_y_dx in range(ite2):
        #                         motion.end_effector_set()
        #                         pose = motion.print_current_pose()
        #                         euler = copy.deepcopy(default_ori)
        #                         for theta_x_dx in range(ite2):
        #                             euler[0] = 2 * np.pi / ite2 * theta_x_dx - np.pi
        #                             euler[1] = 2 * np.pi / ite2 * theta_y_dx - np.pi
        #                             q = motion.euler_to_quaternion(euler = euler) 
        #                             rot_success = motion.rot_motion(pose = pose.position, q = q)
        #                             print("############### second motion:", rot_success, ", ", theta_y_dx *ite2 + theta_x_dx + theta1_dx * ite2**2 + 1, "/", ite2**2*ite1)
        #                             success_data = np.array([[success, rot_success]]) if success_data.shape == (0,) else np.concatenate([success_data, np.array([[success, rot_success]])], 0)
        #                 else:
        #                     for failuer in range(ite2**2):
        #                         success_data = np.array([[success, False]]) if success_data.shape == (0,) else np.concatenate([success_data, np.array([[success, False]])], 0)
                
        #         motion.save_array(data_name="goal_pose", data=goal_data, height=height, r_scale=r_scale)
        #         motion.save_array(data_name="actual_pose", data=actual_data, height=height, r_scale=r_scale)
        #         motion.save_array(data_name="success", data=success_data, height=height, r_scale=r_scale)
        #         #motion.print_current_pose()
        #         r_scale += r_dx * 0.2
        #     height += 0.05 * h_dx
        #     r_scale = 0



        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
    #rrtconect