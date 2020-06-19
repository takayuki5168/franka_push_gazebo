#!/usr/bin/env python
from IPython import embed
import math
import sys
import signal
import time
from subprocess import call
import os, glob

import rospy
from std_msgs.msg import Float32MultiArray, Float64
from gazebo_msgs.msg import ModelState, LinkStates
from gazebo_msgs.srv import GetModelState, SetModelState, GetLinkProperties, SetLinkProperties
from geometry_msgs.msg import Pose, Point, Quaternion
import moveit_commander

class GazeboInterface(object):
    def __init__(self, no_panda=False):
        #rospy.init_node('gazebo_interface')

        # name
        self.robot_name = "panda"
        self.robot_hand_name = "panda_hand"
        self.object_name = "object"
        self.plane_name = "plane"
        #self.origin_name = self.robot_name
        self.origin_name = "world"

        self.mass = 0.1
        self.izz = 0.001
        self.mu = [0.1, 0.1]


        # robot commander
        if not no_panda:
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

        # wait /gazebo/get_model_state
        rospy.wait_for_service('/gazebo/get_model_state')

        # initial robot pose
        self.initial_joint_angles = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

        # initial object pose
        #self.initial_object_pose = [0.5, 0, 0]

        # for plane state
        self.plane_state_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.plane_state_cb)
        self.plane_z_controller_pub = rospy.Publisher("/plane/plane_z_joint_position_controller/command", Float64, queue_size=1)
        self.plane_x_controller_pub = rospy.Publisher("/plane/plane_x_joint_position_controller/command", Float64, queue_size=1)
        self.plane_board_controller_pub = rospy.Publisher("/plane/plane_board_joint_position_controller/command", Float64, queue_size=1)

        self.plane_pose = Pose()

        # object params
        self.object_params_pub = rospy.Publisher("object_plugin/params", Float32MultiArray, queue_size=1)

        # delete gazebo defaulg log
        f_name = glob.glob(os.environ['HOME'] + "/.gazebo/server*")[0] + "/default.log"
        call(["rm", f_name])

    def plane_state_cb(self, msg):
        self.plane_pose = msg.pose[msg.name.index("plane::plane_board_link")]

    def sgn(self, a):
        return 1 if a > 0 else -1

    # get object state
    def get_object_state(self):
        get_object_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        return get_object_state(self.object_name, self.origin_name)

    def get_object_pose(self):
        return self.get_object_state().pose

    def get_object_z(self):
        pose = self.get_object_pose().position
        return pose.z

    def get_object_pos(self):
        pose = self.get_object_pose().position
        return [pose.x, pose.y]

    def get_object_ori(self):
        orientation = self.get_object_pose().orientation
        theta = self.sgn(orientation.z) * math.acos(orientation.w) * 2
        return theta

    def print_object_pose(self):
        print(self.get_object_pose())

    def get_object_prop(self):
        get_object_prop = rospy.ServiceProxy('/gazebo/get_link_properties', GetLinkProperties)
        return get_object_prop(self.object_name + "::link")

    def get_object_cog(self):
        cog = self.get_object_prop().com.position
        return [cog.x, cog.y, cog.z]

    def get_object_mass(self):
        mass = self.get_object_prop().mass
        izz = self.get_object_prop().izz
        return mass, izz

    def print_object_prop(self):
        mass, izz = self.get_object_mass()
        print('  mass:{}'.format(mass))
        print('  izz:{}'.format(izz))
        print('  mu:{}, mu2:{}'.format(self.mu[0], self.mu[1]))


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

    def set_object_pose(self, x, y, z, theta):
         state_msg = ModelState()
         state_msg.pose = Pose(position=Point(*[x, y, z]),
                               orientation=Quaternion(*[0, 0, math.sin(theta / 2.), math.cos(theta / 2.)]))
         self.set_object_state(state_msg)

    # def init_object_pose(self):
    #     self.set_object_pose(*self.initial_object_pose)

    def set_object_params(self, mass=0.1, izz=0.001, mu=[0.1, 0.1]):
        self.mass = mass
        self.izz = izz
        self.mu = mu

        msg = Float32MultiArray(data=[mass, izz] + mu)
        self.object_params_pub.publish(msg)

    # def set_object_mass(self, mass, izz=None):
    #     set_object_prop = rospy.ServiceProxy('/gazebo/set_link_properties', SetLinkProperties)
    #     if izz:
    #         return set_object_prop(link_name=self.object_name+"::link", mass=mass)
    #     else:
    #         return set_object_prop(link_name=self.object_name+"::link", mass=mass, izz=izz)

    # def set_object_prop(self, mass, cog):
    #     set_object_prop = rospy.ServiceProxy('/gazebo/set_link_properties', SetLinkProperties)
    #     return set_object_prop(link_name=self.object_name+"::link", mass=mass, com=Pose(position=Point(x=cog[0], y=cog[1], z=0)),
    #                            ixx=0.0014, iyy=0.0014, izz=0.0014,
    #                            gravity_mode=True)

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
    def set_joint_angles(self, joint_angles, vel=1):
        self.group.set_joint_value_target(joint_angles)
        self.group.set_start_state_to_current_state()
        plan = self.group.plan()
        plan = self.group.retime_trajectory(self.robot.get_current_state(), plan, vel)

        self.group.execute(plan)

    def init_robot_pose(self):
        self.set_joint_angles(self.initial_joint_angles)

    def set_cartesian_pose(self, pos=[0, 0, 0], quat=[1, 0, 0, 0], vel=1, wait=True):
        pose = Pose(position=Point(*pos), orientation=Quaternion(*quat))

        #for i in range(5):
        self.group.set_pose_target(pose, self.robot_hand_name)
        plan = self.group.plan()
        plan = self.group.retime_trajectory(self.robot.get_current_state(), plan, vel)

        try:
            elapsed_time = plan.joint_trajectory.points[-1].time_from_start - plan.joint_trajectory.points[0].time_from_start
        except:
            #if i == 4:
            return -1
            #call(["pkill", "rosmaster", "-9"])
            #call(["pkill", "gzserver", "-9"])
            #sys.exit(1)

        elapsed_time_sec = elapsed_time.secs + elapsed_time.nsecs * 1e-9
        print("elapsed_time:{}".format(elapsed_time_sec))

        if elapsed_time_sec < 5:
            self.group.execute(plan, wait)

        return elapsed_time_sec

    # get plane state
    def get_plane_pose(self):
        return self.plane_pose

    def get_plane_pos(self):
        pose = self.get_plane_pose().position
        return [pose.x, pose.y]

    def get_plane_ori(self):
        orientation = self.get_plane_pose().orientation
        theta = self.sgn(orientation.z) * math.acos(orientation.w) * 2
        return theta

    def print_plane_pose(self):
        print(self.get_plane_pose())

    # set plane state
    def set_plane_inclination(self, direction=[0, 0], angle=0):
        if direction == [0, 0]:
            direction = [1, 0]
            angle = 0

        angle_z = math.atan2(direction[1], direction[0]) - math.pi / 2
        sgn = 1
        while angle_z > math.pi / 2:
            angle_z -= math.pi
            sgn *= -1
        while angle_z < -math.pi / 2:
            angle_z += math.pi
            sgn *= -1
        msg_z = Float64(data=angle_z)

        angle_x = sgn * -angle
        msg_x = Float64(data=angle_x)

        angle_z2 = 2 * math.asin(math.cos(angle_x) * math.sin(angle_z / 2.))
        angle_z2 = abs(angle_z2) * -1 * self.sgn(angle_z)
        #print(angle_z2)
        msg_z2 = Float64(data=angle_z2)

        self.plane_z_controller_pub.publish(msg_z)
        self.plane_x_controller_pub.publish(msg_x)
        self.plane_board_controller_pub.publish(msg_z2)

        print(angle_z, angle_x, angle_z2)


if __name__ == '__main__':
    rospy.init_node('gazebo_interface')

    no_panda = True if "--np" in sys.argv else False

    signal.signal(signal.SIGINT, lambda x, y: sys.exit(0))
    gi = GazeboInterface(no_panda)
    embed()
