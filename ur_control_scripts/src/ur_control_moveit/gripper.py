"""
Copyright (c) 2018-2021 Cristian Beltran
https://github.com/cambel/ur3/blob/noetic-devel/LICENSE

Modifications copyright (C) 2022 Shumpe MORITA.
"""
from __future__ import print_function
from six.moves import input

import sys
import rospy
import actionlib
from std_msgs.msg import Float64
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
# Link attacher
try:
    from gazebo_ros_link_attacher.srv import Attach, AttachRequest
except ImportError:
    print("Grasping pluging can't be loaded")


class Gripper_control(object):
    def __init__(self):
        super(Gripper_control, self).__init__()
        self.client = actionlib.SimpleActionClient("/gripper_controller/gripper_cmd",GripperCommandAction)
        self.clear() 

        if not self.client.wait_for_server(rospy.Duration(5.0)):
            print("/gripper_controller/gripper_cmd not found.")
            sys.exit(1)

        self.attach_link = 'robot::wrist_3_link'
        attach_plugin = rospy.get_param("grasp_plugin", default=False)
        if attach_plugin:
            try:
                # gazebo_ros link attacher
                self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
                self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
                rospy.logdebug('Waiting for service: {0}'.format(self.attach_srv.resolved_name))
                rospy.logdebug('Waiting for service: {0}'.format(self.detach_srv.resolved_name))
                self.attach_srv.wait_for_service()
                self.detach_srv.wait_for_service()
            except Exception:
                rospy.logerr("Fail to load grasp plugin services. Make sure to launch the right Gazebo world!")
            

    def gripper_command(self, position, effort):
        """
        Opens and closes the gripper.
        The command is to move each finger by how many meters.
        Max gripper spacing is 0.05 [m]

        Parameters
        ----------
        position : float
            Distance traveled by each finger. range (0 ~ 0.025)
        effort : float
            The force value to send to the joint.
        """
        self.goal.command.position = position 
        self.goal.command.max_effort = effort 
        self.client.send_goal(self.goal)

    def grab(self, link_name):
        """
        Connect links with Gazebo.

        Parameters
        ----------
        link_name : str
            Model link name.

        Returns
        -------
        res.ok : bool
            Whether it can link.
        """
        parent = self.attach_link.split('::')
        child = link_name.split('::')
        req = AttachRequest()
        req.model_name_1 = parent[0]
        req.link_name_1 = parent[1]
        req.model_name_2 = child[0]
        req.link_name_2 = child[1]
        res = self.attach_srv.call(req)
        return res.ok

    def release(self, link_name):
        """
        Break the link in Gazebo.

        Parameters
        ----------
        link_name : str
            Model link name.

        Returns
        -------
        res.ok : bool
            Whether it can link.
        """
        parent = self.attach_link.rsplit('::')
        child = link_name.rsplit('::')
        req = AttachRequest()
        req.model_name_1 = parent[0]
        req.link_name_1 = parent[1]
        req.model_name_2 = child[0]
        req.link_name_2 = child[1]
        res = self.detach_srv.call(req)
        return res.ok

    def wait(self, timeout=0.1):
        self.client.wait_for_result(timeout=rospy.Duration(timeout))
        return self.client.get_result() 

    def clear(self):
        self.goal = GripperCommandGoal()