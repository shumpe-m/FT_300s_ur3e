import rospy 
import rospkg 
import tf
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
import numpy as np


class cube_control():
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
        self.dish1_center = [0.0925, 0.35, 0.79]
        # dish2
        self.dish2_center = [-0.1075, 0.35, 0.79]


    def pose_control(self, model_name = "cube1", range = "dish1"):
        rand = np.random.rand(3)
        state_msg = ModelState()
        state_msg.model_name = model_name

        # workspace
        if range == "workspace":
            state_msg.pose.position.x = rand[0] * (abs(self.min_x) + abs(self.max_x)) + self.min_x
            state_msg.pose.position.y = rand[1] * (abs(self.min_y) + abs(self.max_y)) + self.min_y
        # dish 1
        elif range == "dish1":
            state_msg.pose.position.x = np.cos(rand[0] * 2 * np.pi) * (self.dish_radius - 0.025) + self.dish1_center[0]
            state_msg.pose.position.y = np.sin(rand[1] * 2 * np.pi) * (self.dish_radius - 0.025) + self.dish1_center[1]
        # dish 2
        elif range == "dish2":
            state_msg.pose.position.x = np.cos(rand[0] * 2 * np.pi) * (self.dish_radius - 0.025) + self.dish2_center[0]
            state_msg.pose.position.y = np.sin(rand[1] * 2 * np.pi) * (self.dish_radius - 0.025) + self.dish2_center[1]
        # dish 3
        elif range == "dish3":
            state_msg.pose.position.x = np.cos(rand[0] * 2 * np.pi) * (self.dish_radius - 0.025) + self.dish1_center[0]
            state_msg.pose.position.y = np.sin(rand[1] * 2 * np.pi) * (self.dish_radius - 0.025) - self.dish1_center[1]
        # dish 4
        elif range == "dish4":
            state_msg.pose.position.x = np.cos(rand[0] * 2 * np.pi) * (self.dish_radius - 0.025) + self.dish2_center[0]
            state_msg.pose.position.y = np.sin(rand[1] * 2 * np.pi) * (self.dish_radius - 0.025) - self.dish2_center[1]
        else:
            print("Input Error: 'range'")
    
        state_msg.pose.position.z = 0.9

        q = tf.transformations.quaternion_from_euler(0, 0, rand[2] * np.pi)
        state_msg.pose.orientation.x = q[0]
        state_msg.pose.orientation.y = q[1]
        state_msg.pose.orientation.z = q[2]
        state_msg.pose.orientation.w = q[3]

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
