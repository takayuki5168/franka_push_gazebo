from gazebo_interface import GazeboInterface

import time
import random
import math

class RandomPose(object):
    def __init__(self):
        self.gi = GazeboInterface()

    def execute(self, iteration=5):
        self.initEnv()
        self.randomizePose()

    def initEnv(self):
        self.gi.init_robot_pose()
        self.gi.init_object_pose()
        time.sleep(0.1)

    def randomizePose(self):
        push_side = random.randint(0, 3)
        push_pos = random.randint(0, 100) / 100.0 - 0.5
        if push_side == 0:
            pre_dx = 0.11
            pre_dy = push_pos * 0.12
            pro_dx = 0.02
            pro_dy = push_pos * 0.12
        elif push_side == 1:
            pre_dx = -0.11
            pre_dy = push_pos * 0.12
            pro_dx = -0.02
            pro_dy = push_pos * 0.12
        elif push_side == 2:
            pre_dx = push_pos * 0.12
            pre_dy = 0.11
            pro_dx = push_pos * 0.12
            pro_dy = 0.02
        elif push_side == 3:
            pre_dx = push_pos * 0.12
            pre_dy = -0.11
            pro_dx = push_pos * 0.12
            pro_dy = -0.02
        self.gi.set_cartesian_pose([0.6 + pre_dx, pre_dy, 0.16])
        #self.gi.set_cartesian_pose([0.6 + pro_dx, pro_dy, 0.16], [1, 0, 0, 0], 0.1)
        time.sleep(1)

if __name__ == '__main__':
    rp = RandomPose()
    rp.execute()
