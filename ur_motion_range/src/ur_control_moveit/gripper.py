#!/usr/bin/env python
# Python 2 compatibility imports

from __future__ import print_function
from six.moves import input

import sys
import rospy
import actionlib
from std_msgs.msg import Float64
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

        

"""Modifications copyright (C) 2022 Shumpe MORITA."""
class gripper_control(object):
    def __init__(self):
        super(gripper_control, self).__init__()
        self.client = actionlib.SimpleActionClient("/gripper_controller/gripper_cmd",GripperCommandAction)
        self.clear() 

        if not self.client.wait_for_server(rospy.Duration(5.0)):
            print("/gripper_controller/gripper_cmd not found.")
            sys.exit(1)

    def gripper_command(self, position, effort):
        self.goal.command.position = position 
        self.goal.command.max_effort = effort 
        self.client.send_goal(self.goal)

    def wait(self, timeout=0.1):
        self.client.wait_for_result(timeout=rospy.Duration(timeout))
        return self.client.get_result() 

    def clear(self):
        self.goal = GripperCommandGoal()


def main():
    gripper = gripper_control()
    gripper.gripper_command(0,1.0)
    gripper.wait(1.0)


if __name__ == "__main__":
    main()
