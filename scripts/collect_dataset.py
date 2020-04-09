from gazebo_interface import GazeboInterface

import time
import random
import math

class CollectDataset(object):
    def __init__(self):
        self.gi = GazeboInterface()

    def execute(self, iteration=5):
        for i in range(iteration):
            print('{}/{} trial'.format(i + 1, iteration))
            self.initEnv()
            self.randomizeMu()
            self.randomizePush()

    def initEnv(self):
        self.gi.init_robot_pose()
        self.gi.init_object_pose()
        time.sleep(2)

    def randomizeMu(self, min_mu=0.3, max_mu=0.7):
        mu = random.randint(0, 1000) * 1. / 1000 * (max_mu - min_mu) + min_mu
        mu2 = random.randint(0, 1000) * 1. / 1000 * (max_mu - min_mu) + min_mu
        print('  mu:{}, mu2:{}'.format(mu, mu2))
        self.gi.set_object_mu([mu, mu2])
        time.sleep(0.5)

    def randomizePush(self):
        push_angle = random.randint(0, 1000) * 1. / 1000 * math.pi * 2
        print('  push angle:{}'.format(push_angle))
        dx = 0.1 * math.cos(push_angle)
        dy = 0.1 * math.sin(push_angle)
        self.gi.set_cartesian_pose([0.4 + dx, dy, 0.13])
        self.gi.set_cartesian_pose([0.4, 0, 0.13])
        time.sleep(3)

if __name__ == '__main__':
    cd = CollectDataset()
    cd.execute()
