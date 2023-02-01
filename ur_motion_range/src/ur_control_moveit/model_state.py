import rospy
from gazebo_msgs.msg import ModelStates

class model_state(object):
    def __init__(self):
        super(model_state, self).__init__()
        # ros message
        self.sub_vector = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callbackVector)
        self.model_state = ModelStates()


    def callbackVector(self, msg):
        self.model_state = msg


    def get_model_state(self):
        # all information
        while self.model_state.pose == []:
            pass

        return self.model_state


    def get_model_name(self):
        # get object name
        model = self.get_model_state()

        return model.name


    def get_model_pose(self, model_name = "cube1"):
        # get object pose
        self.name = model_name
        model_names = self.get_model_name()
        cube_index = model_names.index(self.name)

        return self.model_state.pose[cube_index]