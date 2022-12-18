#!/usr/bin/env python2
import numpy as np


class FB_Tracking:
    def __init__(self):
        self.mu = 0.5
        self.max_speed = 1.0
        self.k2 = 3
        self.k1 = 1
        self.path_threshold = 0.7

    def plan(self, *args):
        self.now_pos = args[0] # [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        self.goal = args[1] # [x(m), y(m)]

        theta = np.arctan2(self.goal[1] - self.now_pos[1], self.goal[0] - self.now_pos[0])

        self.rho = np.sqrt((self.goal[1]-self.now_pos[1])**2 + (self.goal[0]-self.now_pos[0])**2)
        self.beta = np.arctan2(self.goal[1]-self.now_pos[1], self.goal[0]-self.now_pos[0])
        tmp_angle = self.beta - theta
        self.alpha = np.arctan2(np.sin(tmp_angle), np.cos(tmp_angle))
        self.curvature = 1.0 / self.rho * (self.k2 * (self.alpha-np.arctan(-self.k1 * self.beta)) + \
            (1+self.k1/(1+(self.k1*self.beta)**2))*np.sin(self.alpha))
        vx = self.max_speed *1.0 / (1+self.mu *(self.curvature ** 2))
        vw = vx * self.curvature
        return [vx, vw]
