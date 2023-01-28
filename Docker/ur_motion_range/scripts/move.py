#!/usr/bin/env python
# Python 2 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import time
import copy
import rospy
import geometry_msgs.msg

import arm
import gripper

class ur_control(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(ur_control, self).__init__()
        self.arm_control = arm.arm_control("arm")
        self.gripper_control = gripper.gripper_control("gripper")

        self.goal_positions = [[0.2, 0.45, 0.2],
                             [0.2, 0.25, 0.2],
                             [-0.21, 0.25, 0.2],
                             [-0.21, 0.45, 0.2],
                             [0.2, 0.45, 0.03],
                             [0.2, 0.25, 0.03],
                             [-0.21, 0.25, 0.03],
                             [-0.21, 0.45, 0.03],
                             [-0.21, 0.35, 0.1],
                             [-0.05, 0.35, 0.1],
                             [0.2, 0.35, 0.1],
                             [-0.05, 0.45, 0.1],
                             [-0.05, 0.35, 0.1],
                             [-0.05, 0.25, 0.1]]
 



    def arm_action(self):
        joint_ang = [1.57, -1.57/2, 1.57/2, -1.57, -1.57, 0] #0~3.14, -3.14~0, 
        # motion.setup_bias()
        # self.arm_control.go_to_joint_state(joint_ang)
        # default_pose = self.arm_control.print_current_pose()
        # print(default_pose)
        # success, goal_pose = self.arm_control.go_to_position(position = [-0.05, 0.35, 0.1])
        # print(self.arm_control.quaternion_to_euler(default_pose.orientation))
        # q = self.arm_control.euler_to_quaternion(euler = [3.14, -3.14/4, 0])
        # rot_success = self.arm_control.go_to_pose(pose = [-0.1, 0.4, 0.1], q = q)
        # q = self.arm_control.euler_to_quaternion(euler = [3.14, -3.14/3, 0])
        # rot_success = self.arm_control.go_to_pose(pose = [-0.1, 0.4, 0.1], q = q)
        # rot_success = self.arm_control.go_to_pose(pose = [-0.5, 0.4, 0.1], q = q)

        # euler  default:[3.14, 0, 0]   max+-1.54
        # q = self.arm_control.euler_to_quaternion(euler = [3.14-3.14/8, 0, 0])
        # rot_success = self.arm_control.go_to_pose(pose = self.goal_positions[0], q = q)
        # q = self.arm_control.euler_to_quaternion(euler = [3.14-3.14/2, 0, 0])
        # rot_success = self.arm_control.go_to_pose(pose = self.goal_positions[0], q = q)
        suc_data = []
        for goal_position in self.goal_positions:
            print("-----------------------------------------")
            print(goal_position)
            print("-----------------------------------------")

            self.arm_control.go_to_joint_state(joint_ang)
            default_pose = self.arm_control.print_current_pose()

            suc_n = 0
            for e_idx in range(5):
                e_x = e_idx * 3.14 / 4
                for e_idy in range(5):
                    e_y = e_idy * 3.14 / 4
                    print("e_idx : ", e_idx)
                    print("e_idy : ", e_idy)
                    q = self.arm_control.euler_to_quaternion(euler = [3.14-e_x, 1.57-e_y, 0])
                    rot_success = self.arm_control.go_to_pose(pose = goal_position, q = q)
                    if rot_success == True:
                        suc_n += 1
            suc_data.append(suc_n)
        print("---------------- num of success ----------------")
        print(suc_data)
        print("-----------------------------------------")





    def gripper_action(self):
        # print(motion.print_current_pose())
        # gripper_control.gripper_action(joint_values = [0.015, 0.015])
        self.gripper_control.gripper_open()
        self.gripper_control.gripper_close()


def main():
    try:
        action = ur_control()
        action.arm_action()
        # action.gripper_action()
        

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
    #rrtconect