#!/usr/bin/env python
# Python 2 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import time
import copy
import math

import arm
import gripper

"""Modifications copyright (C) 2022 Shumpe MORITA."""
class ur_control(object):
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
        suc_data = []
        for goal_position in self.goal_positions:
            print("-----------------------------------------")
            print(goal_position)
            print("-----------------------------------------")
            self.arm_control.go_to_joint_state(joint_ang)
            default_pose = self.arm_control.print_current_pose()

            suc_n = 0
            for e_idx in range(5):
                e_x = e_idx * 2 * math.pi / (3 * 4) -  math.pi / (3)
                for e_idy in range(5):
                    print("-----------------------------------------")
                    e_y = e_idy * 2 * math.pi / (3 * 4) -  math.pi / (3)
                    print("e_idx : ", e_idx, e_x)
                    print("e_idy : ", e_idy, e_y)
                    q = self.arm_control.euler_to_quaternion(euler = [3.14 - e_x, 0 - e_y, 0])
                    rot_success = self.arm_control.go_to_pose(pose = goal_position, q = q)
                    if rot_success == True:
                        suc_n += 1
                    print("-----------------------------------------")
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