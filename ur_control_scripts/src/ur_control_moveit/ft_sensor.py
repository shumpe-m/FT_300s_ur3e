import rospy
from geometry_msgs.msg import WrenchStamped

class FT_message(object):
    def __init__(self):
        super(FT_message, self).__init__()
        # ros message
        self.sub_vector = rospy.Subscriber("/wrench", WrenchStamped, self.callbackVector)
        self.ft_message = WrenchStamped()


    def callbackVector(self, msg):
        self.ft_message = msg


    def get_ft_message(self):
        # all information
        while self.ft_message == []:
            pass

        return self.ft_message.wrench
