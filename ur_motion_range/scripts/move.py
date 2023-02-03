#!/usr/bin/env python
# Python 2 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import time
import copy
import math
import rospy

from ur_control_moveit import arm, gripper, cube_control, model_state


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
                    rot_success = self.arm_control.go_to_pose(pose = goal_position, ori = q)
                    if rot_success == True:
                        suc_n += 1
                    print("-----------------------------------------")
            suc_data.append(suc_n)
        print("---------------- num of success ----------------")
        print(suc_data)
        print("-----------------------------------------")


    def pick_and_place(self, name = "dish2"):
        joint_ang = [1.57, -1.57, 1.26, -1.57, -1.57, 0] #0~3.14, -3.14~0, 
        self.arm_control.go_to_joint_state(joint_ang)

        cube_con = cube_control.cube_control()
        cube_con.pose_control(model_name = "cube1", range = name)
        time.sleep(0.2)
        cube_con.pose_control(model_name = "cube2", range = name)
        time.sleep(0.2)
        cube_con.pose_control(model_name = "cube3", range = name)
        time.sleep(0.6)
        model_sta = model_state.model_state()
        cube_pose = model_sta.get_model_pose(model_name = "cube3")
        # print(self.arm_control.print_current_pose())
        # po = self.arm_control.print_current_pose()
        # e = self.arm_control.quaternion_to_euler(quaternion = po.orientation)
        # print(e)

        pick_p = copy.deepcopy(cube_pose)
        # print(pick_p)
        e = self.arm_control.quaternion_to_euler(quaternion = pick_p.orientation)
        q = self.arm_control.euler_to_quaternion(euler = [3.14 + e[0], e[1], e[2]])
        pick_p.position.z = 0.2
        rot_success = self.arm_control.go_to_pose(pose = pick_p, ori = q.tolist())
        self.gripper_control.gripper_command(0.0, 1.0)
        result = self.gripper_control.wait(1.0)
        # print(result)
        pick_p.position.z = cube_pose.position.z - 0.78 + 0.01
        # print(pick_p)
        rot_success = self.arm_control.go_to_pose(pose = pick_p, ori = q.tolist())
        self.gripper_control.gripper_command(0.018, 150.0)
        result = self.gripper_control.wait(1.0)
        # print(result)

        pick_p.position.z = 0.2
        rot_success = self.arm_control.go_to_pose(pose = pick_p, ori = q.tolist())
        self.gripper_control.gripper_command(0.0, 1.0)
        result = self.gripper_control.wait(1.0)
        # print(result)



    def gripper_action(self):
        # print(motion.print_current_pose())
        # gripper_control.gripper_action(joint_values = [0.015, 0.015])
        self.gripper_control.gripper_open()
        self.gripper_control.gripper_close()

    def ex(self):
        # q = self.arm_control.euler_to_quaternion(euler = [3.14, 0, 0])
        # pick_p = [0.2, -0.45, 0.2]
        # rot_success = self.arm_control.go_to_pose(pose = pick_p, ori = q.tolist())

        joint_ang = [1.57, -1.57, 1.26, -1.57, -1.57, 0] #0~3.14, -3.14~0, 
        self.arm_control.go_to_joint_state(joint_ang)

        joint_ang = [1.57/2, -1.57, 1.26, -1.57, -1.57, 0] #0~3.14, -3.14~0, 
        self.arm_control.go_to_joint_state(joint_ang)




def main():
    try:
        action = ur_control()
        # action.arm_action()
        # action.gripper_action()
        for idx in range(10):
            name = "dish2" if idx % 2 == 0 else "dish3" 
            action.pick_and_place(name)
        # action.ex()
        

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()