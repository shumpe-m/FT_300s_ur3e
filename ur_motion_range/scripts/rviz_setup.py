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
# from visualization_msgs.msg import Marker
import json

import numpy as np
try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


class rviz_setup(object):

    def __init__(self, name = 'manipulator'):
        super(rviz_setup, self).__init__()
        self.client = roslibpy.Ros(host='localhost', port=9090)
        self.client.run()
        self.marker_publisher = roslibpy.Topic(client, '/mesh_marker', 'visualization_msgs/Marker')
        # # Mesh file message
        # rospy.init_node('mesh_publisher')
        # self.mesh_pub = rospy.Publisher('mesh_marker', Marker, queue_size=10)

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




    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        box_name = self.box_name
        scene = self.scene

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def add_box(self, name = "", pose = [0, 0, 0], size = [0.1, 0.1, 0.1], timeout=4):
        box_name = name
        scene = self.scene

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        #box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = pose[0]
        box_pose.pose.position.y = pose[1]
        box_pose.pose.position.z = pose[2]
        scene.add_box(box_name, box_pose, size=(size[0], size[1], size[2]))

        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def add_mesh(self, marker):
        self.marker_publisher.publish(json.dumps(marker))
        self.client.terminate()




def main():
    try:
        print("----------------------------------------------------------")
        motion = rviz_setup("arm")

        motion.add_box(name = "base_box", pose = [0, 0, 0.05], size = [0.9, 0.12, 0.099])
        motion.add_box(name = "table", pose = [0, 0, -0.0025], size = [0.9, 1.2, 0.005])

        marker = {
            "header": {
                "frame_id": "world"
            },
            "type": "mesh_resource",
            "mesh_resource": "package://your_package_name/path/to/mesh_file.dae",
            "action": "add",
            "scale": {
                "x": 1,
                "y": 1,
                "z": 1
            },
            "color": {
                "r": 1.0,
                "g": 0.0,
                "b": 0.0,
                "a": 1.0
            }
        }
        


    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
    #rrtconect