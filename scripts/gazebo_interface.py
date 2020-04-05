from IPython import embed

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from geometry_msgs.msg import Pose

class GazeboInterface(object):
    def __init__(self):
        rospy.init_node('gazebo_interface')

        # name
        self.robot_name = "panda"
        self.object_name = "wood_cube_10cm"
        self.origin_name = self.robot_name

        # initial object pose
        self.initial_object_pose = Pose()
        self.initial_object_pose.position.x = 0.4
        self.initial_object_pose.position.y = 0
        self.initial_object_pose.position.z = 0

    # get
    def get_object_state(self):
        try:
            get_object_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            return get_object_state(self.object_name, self.origin_name)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def get_object_pose(self):
        return self.get_object_state().pose

    def print_object_pose(self):
        print(self.get_object_pose())

    # set
    def set_object_state(self, state):
        state_msg = ModelState()
        state_msg.model_name = self.object_name
        state_msg.pose = state.pose
        state_msg.twist = state.twist

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_object_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            res = set_object_state(state_msg)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def set_object_pose(self, pose):
        state_msg = ModelState()
        state_msg.pose = pose
        self.set_object_state(state_msg)

    def reset_object_pose(self):
        self.set_object_pose(self.initial_object_pose)

if __name__ == '__main__':
    gi = GazeboInterface()
    embed()
