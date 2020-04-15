from IPython import embed

import rospy
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from geometry_msgs.msg import Pose, Point, Quaternion
import moveit_commander

class GazeboInterface(object):
    def __init__(self):
        rospy.init_node('gazebo_interface')

        # name
        self.robot_name = "panda"
        self.robot_hand_name = "panda_hand"
        self.object_name = "object"
        self.origin_name = self.robot_name

        # moveit commander
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("panda_arm")

        # initial robot pose
        self.initial_joint_angles = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

        # initial object pose
        self.initial_object_pose = Pose()
        self.initial_object_pose.position.x = 0.6
        self.initial_object_pose.position.y = 0
        self.initial_object_pose.position.z = 0

        # object params
        self.object_mu_pub = rospy.Publisher("object_params/mu", Float32MultiArray)
        self.object_friction_pub = rospy.Publisher("object_params/friction", Float32MultiArray)

    # get object state
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

    # set object state
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

    def init_object_pose(self):
        self.set_object_pose(self.initial_object_pose)

    # get robot state
    def get_joint_angles(self):
        return self.group.get_current_joint_values()

    def print_joint_angles(self):
        print(self.get_joint_angles())

    def get_cartesian_pose(self):
        return self.group.get_current_pose(self.robot_hand_name)

    def print_cartesian_pose(self):
        print(eslf.get_cartesian_pose())

    # set robot state
    def set_joint_angles(self, joint_angles, vel=2):
        self.group.set_joint_value_target(joint_angles)
        self.group.set_start_state_to_current_state()
        plan = self.group.plan()
        plan = self.group.retime_trajectory(self.robot.get_current_state(), plan, vel)
        self.group.execute(plan)

    def init_robot_pose(self):
        self.set_joint_angles(self.initial_joint_angles)

    def set_cartesian_pose(self, pos=[0, 0, 0], quat=[1, 0, 0, 0], vel=2):
        pose = Pose(position=Point(*pos), orientation=Quaternion(*quat))

        self.group.set_pose_target(pose, self.robot_hand_name)
        plan = self.group.plan()
        plan = self.group.retime_trajectory(self.robot.get_current_state(), plan, vel)
        self.group.execute(plan)

    # set object state
    def set_object_mu(self, mu=[1, 1]):
        msg = Float32MultiArray(data=mu)
        self.object_mu_pub.publish(msg)

    def set_object_friction(self, mu=[1, 1], coef=0):
        fri = mu
        fri.append(coef)
        msg = Float32MultiArray(data=fri)
        self.object_friction_pub.publish(msg)

if __name__ == '__main__':
    gi = GazeboInterface()
    embed()
