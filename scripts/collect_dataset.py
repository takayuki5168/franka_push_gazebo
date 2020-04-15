from gazebo_interface import GazeboInterface

import time
import random
import math
import datetime

import tf
from geometry_msgs.msg import Vector3

class CollectDataset(object):
    def __init__(self):
        self.gi = GazeboInterface()
        self.file_name = "{0:%Y-%m%d-%H%M}.log".format(datetime.datetime.now())

    def execute(self, iteration=5):
        self.initMu(0.1)
        for i in range(iteration):
            print('{}/{} trial'.format(i + 1, iteration))
            self.initEnv()
            #self.randomizeMu()
            self.randomizePush()
            self.record()

    def initMu(self, mu=0.1):
        print('  mu:{}, mu2:{}'.format(mu, mu))
        self.gi.set_object_mu([mu, mu])
        time.sleep(0.1)

    def initEnv(self):
        self.gi.init_robot_pose()
        self.gi.init_object_pose()
        time.sleep(0.1)

    def sgn(self, a):
        return 1 if a > 0 else -1

    def record(self):
        pose = self.gi.get_object_pose()
        dx = pose.position.x - 0.6
        dy = pose.position.y
        dtheta = self.sgn(pose.orientation.z) * math.acos(pose.orientation.w) * 2

        print(dtheta * 180 / math.pi)
        print('{} {} {} {} {}\n'.format(self.push_side, self.push_pos, dx, dy, dtheta))

        self.log_file = open(self.file_name, 'a')
        self.log_file.write('{} {} {} {} {}\n'.format(self.push_side, self.push_pos, dx, dy, dtheta))
        self.log_file.close()

    def randomizeMu(self, min_mu=0.1, max_mu=0.1):
        mu = random.randint(0, 1000) * 1. / 1000 * (max_mu - min_mu) + min_mu * 2
        mu2 = random.randint(0, 1000) * 1. / 1000 * (max_mu - min_mu) + min_mu * 2
        print('  mu:{}, mu2:{}'.format(mu, mu2))
        self.gi.set_object_mu([mu, mu2])
        time.sleep(0.1)

    def oldRandomizeMu(self, min_mu=0.1, max_mu=0.3):
        mu = random.randint(0, 1000) * 1. / 1000 * (max_mu - min_mu) + min_mu
        mu2 = random.randint(0, 1000) * 1. / 1000 * (max_mu - min_mu) + min_mu
        print('  mu:{}, mu2:{}'.format(mu, mu2))
        self.gi.set_object_mu([mu, mu2])
        time.sleep(0.1)

    def randomizePush(self):
        push_side = random.randint(0, 3)
        push_pos = random.randint(0, 100) / 100.0 - 0.5
        if push_side == 0:
            pre_dx = 0.09
            pre_dy = push_pos * 0.12
            pro_dx = 0.0
            pro_dy = push_pos * 0.12
        elif push_side == 1:
            pre_dx = -0.09
            pre_dy = push_pos * 0.12
            pro_dx = -0.0
            pro_dy = push_pos * 0.12
        elif push_side == 2:
            pre_dx = push_pos * 0.12
            pre_dy = 0.09
            pro_dx = push_pos * 0.12
            pro_dy = 0.0
        elif push_side == 3:
            pre_dx = push_pos * 0.12
            pre_dy = -0.09
            pro_dx = push_pos * 0.12
            pro_dy = -0.0
        #push_angle = random.randint(0, 10) * 1. / 10 * math.pi
        #print('  push angle:{}'.format(push_angle))
        self.gi.set_cartesian_pose([0.6 + pre_dx, pre_dy, 0.16])
        self.gi.set_cartesian_pose([0.6 + pro_dx, pro_dy, 0.16], [1, 0, 0, 0], 0.1)
        time.sleep(1)

        self.push_side = push_side
        self.push_pos = push_pos


    def oldRandomizePush(self):
        push_angle = random.randint(0, 1000) * 1. / 1000 * math.pi * 2
        print('  push angle:{}'.format(push_angle))
        dx = 0.1 * math.cos(push_angle)
        dy = 0.1 * math.sin(push_angle)
        self.gi.set_cartesian_pose([0.6 + dx, dy, 0.13])
        self.gi.set_cartesian_pose([0.6, 0, 0.13])
        time.sleep(3)

if __name__ == '__main__':
    cd = CollectDataset()
    cd.execute(500)
