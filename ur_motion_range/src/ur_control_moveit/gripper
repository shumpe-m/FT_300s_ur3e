#!/usr/bin/env python
# Python 2 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import time
import copy
import tf
import rospy
import moveit_commander
import moveit_msgs.msg
try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt
    tau = 2.0 * pi
    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))
        

class gripper_control(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self, name = 'gripper'):
        super(gripper_control, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("ur_planner", anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.name = name
        self.move_group = moveit_commander.MoveGroupCommander(self.name)
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        self.planning_frame = self.move_group.get_planning_frame()
        self.group_names = self.robot.get_group_names()


    def gripper_action(self, joint_values = [0,0]):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal = joint_values

        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return current_joints

    def gripper_open(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal = [0, 0]

        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return current_joints

    def gripper_close(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal = [0.025, 0.025]

        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return current_joints

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)

    def print_current_pose(self):
        move_group = self.move_group
        joint_values = move_group.get_current_joint_values()
        return joint_values



def main():
    try:
        motion = gripper_control("gripper")
        # print(motion.print_current_pose())
        motion.gripper_action(joint_values = [0.015, 0.015])
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
    #rrtconect