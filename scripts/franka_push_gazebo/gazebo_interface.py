#!/usr/bin/env python
from IPython import embed
import math
import sys
import signal

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

        # robot commander
        rd = False
        while not rd:
            rd = rospy.get_param('robot_description', False)
        self.robot = moveit_commander.RobotCommander()

        # move group commander
        is_move_group = False
        while not is_move_group:
            try:
                self.group = moveit_commander.MoveGroupCommander("panda_arm")
                is_move_group = True
            except:
                continue

        # initial robot pose
        self.initial_joint_angles = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

        # initial object pose
        self.initial_object_pose = [0.6, 0, 0]

        # object params
        self.object_params_pub = rospy.Publisher("object_plugin/params", Float32MultiArray)
        #self.object_friction_pub = rospy.Publisher("object_params/friction", Float32MultiArray)

    def sgn(self, a):
        return 1 if a > 0 else -1

    # get object state
    def get_object_state(self):
        try:
            get_object_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            return get_object_state(self.object_name, self.origin_name)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def get_object_pose(self):
        return self.get_object_state().pose

    def get_object_pos(self):
        pose = self.get_object_pose().position
        return [pose.x, pose.y]

    def get_object_ori(self):
        orientation = self.get_object_pose().orientation
        theta = self.sgn(orientation.z) * math.acos(orientation.w) * 2
        return theta

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

    # def set_object_pose(self, pose):
    #     state_msg = ModelState()
    #     state_msg.pose = pose
    #     self.set_object_state(state_msg)

    def set_object_pose(self, x, y, theta):
         state_msg = ModelState()
         state_msg.pose = Pose(position=Point(*[x, y, 0]),
                               orientation=Quaternion(*[0, 0, math.sin(theta / 2.), math.cos(theta / 2.)]))
         self.set_object_state(state_msg)

    def init_object_pose(self):
        self.set_object_pose(*self.initial_object_pose)

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
    def set_object_params(self, mass=1.0, cog=[0, 0], mu=[1, 1]):
        msg = Float32MultiArray(data=[mass] + cog + mu)
        self.object_params_pub.publish(msg)

    # def set_object_friction(self, mu=[1, 1], coef=0):
    #     fri = mu
    #     fri.append(coef)
    #     msg = Float32MultiArray(data=fri)
    #     self.object_friction_pub.publish(msg)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, lambda x, y: sys.exit(0))
    gi = GazeboInterface()
    embed()
