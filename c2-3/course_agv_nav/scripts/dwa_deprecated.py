#!/usr/bin/env python2
import numpy as np
import matplotlib.pyplot as plt

class Config:
    def __init__(self):
        self.min_speed = -0.5
        self.max_speed = 1  # [m/s]
        self.max_yawrate = 2.5  # [rad/s]
        self.max_accel = 0.5  # [m/s2]
        self.max_dyawrate = 1  # [rad/s2]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.v_reso = self.max_accel*self.dt/2.0  # [m/s]
        # self.v_reso = 0.02
        self.yawrate_reso = self.max_dyawrate*self.dt/2.0  # [rad/s]
        self.predict_time = 2  # [s]
        self.alpha = 0.25 # heading
        self.beta = 0.55 # dist
        self.gamma =  0.2 # velocity
        self.robot_radius = 0.3 #0.37  # [m] for collision check
        self.laser_threshold = 1.5


class DWAPlanner:
    def __init__(self):
        self.config = Config()

    def plan(self, plan_x, plan_goal, plan_ob):
        self.now_pos = plan_x # [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        self.goal = plan_goal
        self.ob = plan_ob
        best_velocity = self.dwa_plan()
        return best_velocity

    def velocity_dynamic_window(self):
        # Max Speed
        Vs = [self.config.min_speed, self.config.max_speed,
            -self.config.max_yawrate, self.config.max_yawrate]

        # Dynamic window from motion model
        Vd = [self.now_pos[3] - self.config.max_accel * self.config.dt,
            self.now_pos[3] + self.config.max_accel * self.config.dt,
            self.now_pos[4] - self.config.max_dyawrate * self.config.dt,
            self.now_pos[4] + self.config.max_dyawrate * self.config.dt]

        #  [vmin, vmax, yaw_rate min, yaw_rate max]
        v_union = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
            max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

        return v_union

    def calc_motion(self, x, v):
        dt = self.config.dt
        x[0] += v[0] * dt * np.cos(x[2])
        x[1] += v[0] * dt * np.sin(x[2])
        x[2] += v[1] * dt
        x[3] = v[0]
        x[4] = v[1]
        ########## Method 2
        # r = v[0] * 1.0 / v[1]
        # next_theta = x[2] + v[1] * dt
        # x[0] = x[0] + r*np.sin(x[2]) - r * np.sin(next_theta)
        # x[1] = x[1] - r*np.cos(x[2]) + r * np.cos(next_theta)
        return x
    
    def calc_trajectory(self, step_v, step_w): #calc sim trajectory
        x = np.array(self.now_pos)
        traj = np.array(self.now_pos)
        time = 0
        while time <= self.config.predict_time:
            x = self.calc_motion(x, [step_v, step_w])
            traj = np.vstack((traj, x))
            time += self.config.dt
        return traj

    def dwa_plan(self):
        dw = self.velocity_dynamic_window() # velocity window
        best_velocity = [0.0, 0.0]
        best_trajectory = np.array([self.now_pos])
        feasible = []

        # evaluate all trajectory with sampled input in dynamic window
        for step_v in np.arange(dw[0], dw[1], self.config.v_reso):
            for step_w in np.arange(dw[2], dw[3], self.config.yawrate_reso):
                # print(step_v, step_w)
                trajectory = self.calc_trajectory(step_v, step_w)
                # calc cost
                heading = self.heading_cost(trajectory)
                dist = self.obstacle_cost(trajectory)
                velocity = self.velocity_cost(trajectory)
                if dist > 0:
                    feasible.append(np.array([heading, dist, velocity, step_v, step_w]))

        # norm the cost
        feasible = np.array(feasible)
        feasible[:,0] = feasible[:,0] / np.sum(feasible[:,0])
        feasible[:,1] = feasible[:,1] / np.sum(feasible[:,1])
        feasible[:,2] = feasible[:,2] / np.sum(feasible[:,2])

        max_cost = 0.0
        for i in range(feasible.shape[0]):
            final_cost = self.config.alpha*feasible[i,0] + self.config.beta*feasible[i,1] \
                + self.config.gamma * feasible[i,2]
            if final_cost > max_cost:
                max_cost = final_cost
                best_velocity = [feasible[i,3], feasible[i,4]]

        print(best_velocity)
        return best_velocity

    def heading_cost(self, trajectory):
        dx = self.goal[0] - trajectory[-1, 0]
        dy = self.goal[1] - trajectory[-1, 1]
        cost_angle = np.arctan2(dy, dx) - trajectory[-1, 2]
        cost = 180 - abs(np.arctan2(np.sin(cost_angle), np.cos(cost_angle))) * 180.0 / np.pi
        return cost

    def velocity_cost(self, trajectory):
        return abs(trajectory[-1, 3])

    def obstacle_cost(self, trajectory):
        # self.filter_thres = 0.03
        # self.ob = self.ob[(self.ob[:,0]>self.filter_thres) | (self.ob[:,1]>self.filter_thres)]
        ox = self.ob[:, 0]
        oy = self.ob[:, 1]
        # plt.scatter(ox, oy)
        # dx = ox
        # dy = oy
        dx = trajectory[:, 0] - ox[:, None]
        dy = trajectory[:, 1] - oy[:, None]
        # plt.scatter(trajectory[:,0], trajectory[:,1])
        # plt.show()
        r = np.hypot(dx, dy)        
        if (r <= self.config.robot_radius).any():
            return -1
        # plt.scatter(ox, oy)
        # plt.scatter(trajectory[:,0], trajectory[:,1])
        # plt.show()
        min_r = np.min(r)
        # print(min_r)
        if min_r > 3 * self.config.robot_radius:
            min_r = 3 * self.config.robot_radius
        return min_r
