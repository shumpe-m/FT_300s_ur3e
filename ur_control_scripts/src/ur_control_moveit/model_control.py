#!/usr/bin/env python
# Python 2 compatibility imports
import rospy 
import rospkg 
import tf
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
import numpy as np


class Box_control():
    def __init__(self):
        # workspace
        self.max_x = 0.2
        self.min_x = -0.21
        self.max_y = 0.45
        self.min_y = 0.25
        self.max_z = 0.2
        self.min_z = 0.01 + 0.02 # 0.02=offset
        # dish radius
        self.dish_radius = 0.09
        # dish1
        self.dish1_center = [0.0925, 0.35, 0.7801]
        # dish2 = bin1
        self.dish2_center = [-0.1075, 0.35, 0.7801]
        # dish3 = bin2
        self.dish3_center = [0.0935, -0.35, 0.7801]
        # dish4 = dish2
        self.dish4_center = [-0.1065, -0.35, 0.7801]


    def radom_pose(self, model_name = "box1", area = "dish1", shouldChange = True):
        rand = np.random.rand(3)
        state_msg = ModelState()
        state_msg.model_name = model_name

        # workspace
        if area == "workspace":
            state_msg.pose.position.x = rand[0] * (abs(self.min_x) + abs(self.max_x)) + self.min_x
            state_msg.pose.position.y = rand[1] * (abs(self.min_y) + abs(self.max_y)) + self.min_y
        # dish 1
        elif area == "dish1":
            state_msg.pose.position.x = np.cos(rand[0] * 2 * np.pi) * (self.dish_radius - 0.025) + self.dish1_center[0]
            state_msg.pose.position.y = np.sin(rand[1] * 2 * np.pi) * (self.dish_radius - 0.025) + self.dish1_center[1]
        # bin 1
        elif area == "bin1":
            state_msg.pose.position.x = np.cos(rand[0] * 2 * np.pi) * (self.dish_radius - 0.025) + self.dish2_center[0]
            state_msg.pose.position.y = np.sin(rand[1] * 2 * np.pi) * (self.dish_radius - 0.025) + self.dish2_center[1]
        # dish 2
        elif area == "dish2":
            state_msg.pose.position.x = np.cos(rand[0] * 2 * np.pi) * (self.dish_radius - 0.025) + self.dish4_center[0]
            state_msg.pose.position.y = np.sin(rand[1] * 2 * np.pi) * (self.dish_radius - 0.025) + self.dish4_center[1]
        # bin 2
        elif area == "bin2":
            state_msg.pose.position.x = np.cos(rand[0] * 2 * np.pi) * (self.dish_radius - 0.025) + self.dish3_center[0]
            state_msg.pose.position.y = np.sin(rand[1] * 2 * np.pi) * (self.dish_radius - 0.025) + self.dish3_center[1]
        else:
            print("Input Error: 'area'")
        state_msg.pose.position.z = 0.9

        q = tf.transformations.quaternion_from_euler(0, 0, (rand[2] - 0.5) * np.pi + np.pi/2)
        state_msg.pose.orientation.x = q[0]
        state_msg.pose.orientation.y = q[1]
        state_msg.pose.orientation.z = q[2]
        state_msg.pose.orientation.w = q[3]

        if shouldChange:
            rospy.wait_for_service('/gazebo/set_model_state')
            try:
                set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                resp = set_state( state_msg )
            except (rospy.ServiceException, e):
                print("Service call failed: %s" % e)

        return state_msg.pose

class Model_reset():
    def __init__(self):
        self.models = ["box1", "box2", "box3", "jig"]
        self.models_state = [[-0.4, 0.4, 0.8, 0, 0, 0, 1],
                            [-0.35, 0.4, 0.8, 0, 0, 0, 1],
                            [-0.30, 0.4, 0.8, 0, 0, 0, 1],
                            [-0.25, 0.25, 0.8, 0, 0, 0.7071080799, 0.7071054825]]
    
    def reset(self, model_name='all'):
        if model_name == "all":
            for model, model_state in zip(self.models, self.models_state):
                self.model_state_msg(model, model_state)
        else:
            model_index = self.models.index(model_name)
            self.model_state_msg(model_name, self.models_state[model_index])

    def model_state_msg(self, model, model_state):
        state_msg = ModelState()
        state_msg.model_name = model
        state_msg.pose.position.x = model_state[0]
        state_msg.pose.position.y = model_state[1]
        state_msg.pose.position.z = model_state[2]
        state_msg.pose.orientation.x = model_state[3]
        state_msg.pose.orientation.y = model_state[4]
        state_msg.pose.orientation.z = model_state[5]
        state_msg.pose.orientation.w = model_state[6]

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )
        except (rospy.ServiceException, e):
            print("Service call failed: %s" % e)


if __name__ == '__main__':
    try:
        cube_control.pose_control()
    except rospy.ROSInterruptException:
        pass
