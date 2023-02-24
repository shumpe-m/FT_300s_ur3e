#!/usr/bin/env python
# Python 2 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import time
import copy
import math
import rospy

from ur_control_moveit import arm, gripper, model_control, model_state
import rviz_setup


class Ur_control(object):
    def __init__(self):
        super(Ur_control, self).__init__()
        # setup = rviz_setup.Rviz_setup("arm")
        # rviz_setup.main()
        self.arm_control = arm.Arm_control("arm")
        self.gripper_control = gripper.Gripper_control()
        self.box_control = model_control.Box_control()
        self.forward_basic_joint = [0.9419943318766126, -1.4060059478330746, 1.4873566760577779, -1.6507993112633637, -1.5705307531751274, -0.629176246773139] # [1.57, -1.57, 1.26, -1.57, -1.57, 0]
        self.backward_basic_joint = [-2.234010390909684, -1.3927192120354697, 1.472050134256044, -1.65123476873738, -1.5690119071493065, 2.478363663037163] # [-1.57, -1.57, 1.26, -1.57, -1.57, 0]

    #     # test positions
    #     self.goal_positions = [[0.2, 0.45, 0.2],
    #                          [0.2, 0.25, 0.2],
    #                          [-0.21, 0.25, 0.2],
    #                          [-0.21, 0.45, 0.2],
    #                          [0.2, 0.45, 0.03],
    #                          [0.2, 0.25, 0.03],
    #                          [-0.21, 0.25, 0.03],
    #                          [-0.21, 0.45, 0.03],
    #                          [-0.21, 0.35, 0.1],
    #                          [-0.05, 0.35, 0.1],
    #                          [0.2, 0.35, 0.1],
    #                          [-0.05, 0.45, 0.1],
    #                          [-0.05, 0.35, 0.1],
    #                          [-0.05, 0.25, 0.1]]


    # # test function
    # def arm_action_test(self):
    #     joint_ang = [1.57, -1.57/2, 1.57/2, -1.57, -1.57, 0]
    #     suc_data = []
    #     for goal_position in self.goal_positions:
    #         print("-----------------------------------------")
    #         print(goal_position)
    #         print("-----------------------------------------")
    #         self.arm_control.go_to_joint_state(joint_ang)
    #         default_pose = self.arm_control.print_current_pose()

    #         suc_n = 0
    #         for e_idx in range(5):
    #             e_x = e_idx * 2 * math.pi / (3 * 4) -  math.pi / (3)
    #             for e_idy in range(5):
    #                 print("-----------------------------------------")
    #                 e_y = e_idy * 2 * math.pi / (3 * 4) -  math.pi / (3)
    #                 print("e_idx : ", e_idx, e_x)
    #                 print("e_idy : ", e_idy, e_y)
    #                 q = self.arm_control.euler_to_quaternion(euler = [3.14 - e_x, 0 - e_y, 0])
    #                 rot_success = self.arm_control.go_to_pose(pose = goal_position, ori = q)
    #                 if rot_success == True:
    #                     suc_n += 1
    #                 print("-----------------------------------------")
    #         suc_data.append(suc_n)
    #     print("---------------- num of success ----------------")
    #     print(suc_data)
    #     print("-----------------------------------------")



    def pick_and_place(self, area = "dish2"):
        # model name of picking target
        target_model = "box3"

        if area == "dish1" or area == "bin1":
            pick_basic_joint = self.forward_basic_joint
            place_basic_joint = self.backward_basic_joint
        elif area == "dish2" or area == "bin2":
            pick_basic_joint = self.backward_basic_joint
            place_basic_joint = self.forward_basic_joint
        self.arm_control.go_to_joint_state(pick_basic_joint)

        ### Rearrange the box ###
        self.rearrange(area = area)
        model_sta = model_state.Model_state()
        box_pose = model_sta.get_model_pose(model_name = target_model)

        ### Pick ###
        pick_p = copy.deepcopy(box_pose)
        # print(pick_p)
        e = self.arm_control.quaternion_to_euler(quaternion = pick_p.orientation)
        q = self.arm_control.euler_to_quaternion(euler = [3.14 + e[0], e[1], e[2]])
        pick_p.position.z = 0.2
        self.arm_control.go_to_pose(pose = pick_p, ori = q.tolist())
        self.gripper_control.gripper_command(0.0, 1.0)
        result = self.gripper_control.wait(1.0)
        # print(result)
        pick_p.position.z = box_pose.position.z - 0.78 + 0.01
        # print(pick_p)
        pick_success = self.arm_control.go_to_pose(pose = pick_p, ori = q.tolist())
        if pick_success:
            self.gripper_control.gripper_command(0.02, 150.0)
            rospy.sleep(0.5)
            self.gripper_control.grab(link_name=target_model+"::link")

        # pick_p.position.z = 0.2
        # rot_success = self.arm_control.go_to_pose(pose = pick_p, ori = q.tolist())
        # result = self.gripper_control.wait(1.0)
        # print(result)
        self.arm_control.go_to_joint_state(pick_basic_joint)

        ### Place ###
        if pick_success:
            self.arm_control.go_to_joint_state(place_basic_joint)
            if area == "bin1":
                place_area = "dish2"
            elif area == "bin2":
                place_area = "dish1"
            place_pose = self.box_control.radom_pose(model_name = target_model, area = place_area, shouldChange = False)
            place_p = copy.deepcopy(place_pose)
            place_p.position.z = 0.2
            # print([place_p.position.x, place_p.position.y, place_p.position.z])
            rot_success = self.arm_control.go_to_pose(pose = [place_p.position.x, place_p.position.y, place_p.position.z], ori = q.tolist())
            # place_p.position.z = place_pose.position.z - 0.78 + 0.01
            place_p.position.z = 0.03
            rot_success = self.arm_control.go_to_pose(pose = [place_p.position.x, place_p.position.y, place_p.position.z], ori = q.tolist())
            self.gripper_control.gripper_command(0.0, 1.0)
            rospy.sleep(0.2)
            self.gripper_control.release(link_name=target_model+"::link")
            rospy.sleep(1.0)
            self.arm_control.go_to_joint_state(place_basic_joint)
        # self.reset()

    def rearrange(self, area = "bin1"):
        ### Rearrange the box ###
        self.box_control.radom_pose(model_name = "box1", area = area)
        rospy.sleep(0.2)
        self.box_control.radom_pose(model_name = "box2", area = area)
        rospy.sleep(0.2)
        self.box_control.radom_pose(model_name = "box3", area = area)
        rospy.sleep(0.6)

    def gripper_action(self):
        # print(motion.print_current_pose())
        # gripper_control.gripper_action(joint_values = [0.015, 0.015])
        self.gripper_control.gripper_open()
        self.gripper_control.gripper_close()

    def self_reset(self, area=""):
        self.arm_control.go_to_joint_state(self.forward_basic_joint)
        self.gripper_control.gripper_command(0.0, 1.0)
        q = self.arm_control.euler_to_quaternion(euler = [3.14 , 0, 0])
        self.arm_control.go_to_pose(pose = [-0.26, 0.265, 0.2], ori = q.tolist())
        # Grab a jig
        self.arm_control.go_to_pose(pose = [-0.26, 0.265, 0.12], ori = q.tolist())
        self.gripper_control.gripper_command(0.01, 150.0)
        rospy.sleep(0.5)
        self.gripper_control.grab(link_name="jig::link")

        sweep_posi = []
        self.arm_control.go_to_joint_state(self.forward_basic_joint)
        rospy.sleep(0.2)
        if area == "bin1":
            sweep_posi = [[0.2, 0.35, 0.135],
                         [-0.075, 0.35, 0.135],
                         [0.2, 0.27, 0.135],
                         [-0.075, 0.27, 0.135]]
        elif area == "bin2":
            sweep_posi = [[-0.21, -0.35, 0.135],
                         [0.075, -0.35, 0.135],
                         [-0.21, -0.27, 0.135],
                         [0.075, -0.27, 0.135]]
        if sweep_posi != []:
            self.arm_control.go_to_pose(pose = sweep_posi[0], ori = q.tolist())
            self.arm_control.go_to_pose(pose = sweep_posi[1], ori = q.tolist())
            self.arm_control.go_to_joint_state(self.forward_basic_joint)
            rospy.sleep(0.2)
            self.arm_control.go_to_pose(pose = sweep_posi[2], ori = q.tolist())
            self.arm_control.go_to_pose(pose = sweep_posi[3], ori = q.tolist())
            self.arm_control.go_to_joint_state(self.forward_basic_joint)

        # Release a jig
        self.arm_control.go_to_pose(pose = [-0.26, 0.265, 0.2], ori = q.tolist())
        self.arm_control.go_to_pose(pose = [-0.26, 0.265, 0.13], ori = q.tolist())
        self.gripper_control.gripper_command(0.0, 1.0)
        rospy.sleep(0.2)
        self.gripper_control.release(link_name="jig::link")
        rospy.sleep(1.0)
        # Reset jig pose
        self.arm_control.go_to_pose(pose = [-0.26, 0.265, 0.2], ori = q.tolist())
        self.arm_control.go_to_joint_state(self.forward_basic_joint)
        self.reset(model_name="jig")


    def reset(self, model_name="all"):
        model_reset = model_control.Model_reset()
        model_reset.reset(model_name)

    def joint_test(self):
        self.arm_control.go_to_joint_state(self.forward_basic_joint)
        joint = copy.deepcopy(self.forward_basic_joint)
        print(self.arm_control.get_current_joint())
        joint[4] += math.pi / 10 # 1.4
        joint[4] = -1*math.pi  * 0# 1.4
        rospy.sleep(0.2)
        self.arm_control.go_to_joint_state(joint)
        print(self.arm_control.get_current_joint())


def main():
    try:
        action = Ur_control()
<<<<<<< HEAD
        action.reset()
        # action.arm_action_test()
        # action.gripper_action()

        for idx in range(10):
            area = "bin1" if idx % 2 == 0 else "bin2" 
            action.pick_and_place(area)
        action.reset()

        # action.pick_and_place(area = "dish3")

        # action.rearrange("dish1")
        # action.self_reset(area = "bin1")
=======
        
        # pick and place
        # for idx in range(4):
        #     name = "dish2" if idx % 2 == 0 else "dish3" 
        #     action.pick_and_place(name)
        # action.reset()

        # Using jig
        action.rearrange("dish1")
        action.self_reset(area = "bin1")
        action.reset()
>>>>>>> 81d6a3c186abfe11ccb5b1707710580e074a9317

        # action.joint_test()
        

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
