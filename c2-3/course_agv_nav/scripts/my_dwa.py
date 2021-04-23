#!/usr/bin/env python2
import numpy as np
import matplotlib.pyplot as plt

class Config:
    def __init__(self):
        self.min_speed = 0
        self.max_speed = 1  # [m/s]
        self.max_yawrate = 3 #2.5  # [rad/s]
        self.max_accel = 0.5  # [m/s2]
        self.max_dyawrate = 2#1  # [rad/s2]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.v_reso = self.max_accel*self.dt/5.0  # [m/s]
        # self.v_reso = 0.02
        self.yawrate_reso = self.max_dyawrate*self.dt/5.0  # [rad/s]
        self.predict_time = 4  # [s]
        self.alpha = 0.6 # heading
        self.beta = 0.6 # dist
        self.gamma = 1.5 # velocity
        self.robot_radius = 0.33 #0.37  # [m] for collision check
        self.laser_noise = 0.03


class DWAPlanner:
    def __init__(self):
        self.config = Config()
        self.cnt = 5
        self.path_threshold = 1.2

    def plan(self, *args):
        self.now_pos = args[0] # [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        self.goal = args[1]
        plan_ob = args[2]
        self.ob = plan_ob[(abs(plan_ob[:,0])>self.config.laser_noise) | (abs(plan_ob[:,1])>self.config.laser_noise)]
        # plt.scatter(self.ob[:,0], self.ob[:,1])
        # plt.show()
        best_velocity, best_trajectory = self.dwa_plan()
        return best_velocity

    def heading_cost(self, trajectory):
        dx = self.goal[0] - trajectory[-1, 0]
        dy = self.goal[1] - trajectory[-1, 1]
        cost_angle = np.arctan2(dy, dx) - trajectory[-1, 2]
        cost = abs(np.arctan2(np.sin(cost_angle), np.cos(cost_angle))) / (2*np.pi) + \
            np.sqrt(dx**2 + dy**2) / 0.7 # attention
        return cost

    def velocity_cost(self, trajectory):
        return self.config.max_speed - abs(trajectory[-1, 3]) + (self.config.max_yawrate - abs(trajectory[-1, 4]))/self.config.max_yawrate

    def obstacle_cost(self, trajectory):
        ox = self.ob[:, 0]
        oy = self.ob[:, 1]
        dx = trajectory[5::10, 0] - ox[:, None]
        dy = trajectory[5::10, 1] - oy[:, None]
        r = np.hypot(dx, dy)        
        if (r <= self.config.robot_radius).any():
            return float("Inf")
        min_r = np.min(r, axis=0) # each column
        min_r = np.average(min_r)
        if min_r > 3 * self.config.robot_radius:
            min_r = 3 * self.config.robot_radius
        return 1.0 / min_r

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
        
        min_cost = float("inf")
        best_velocity = [0.0, 0.0]
        best_trajectory = np.array([self.now_pos])
        best_cost = [-1,-1,-1]

        # evaluate all trajectory with sampled input in dynamic window
        for step_v in np.arange(dw[0], dw[1], self.config.v_reso):
            for step_w in np.arange(dw[2], dw[3], self.config.yawrate_reso):

                trajectory = self.calc_trajectory(step_v, step_w)
                # calc cost
                heading = self.heading_cost(trajectory)
                velocity = self.velocity_cost(trajectory)
                dist = self.obstacle_cost(trajectory)

                final_cost = self.config.alpha * heading + self.config.beta * dist + self.config.gamma * velocity

                if final_cost < min_cost:
                    min_cost = final_cost
                    best_velocity = [step_v, step_w]
                    best_cost = [heading, velocity, dist]
                    best_trajectory = trajectory
                    self.cnt = min(self.cnt+1, 5)
        # print(best_velocity)
        # print(best_cost)

        if min_cost == float("inf"):
            # print(heading, dist, velocity, self.cnt)
            if(self.cnt > 0):
                self.cnt = self.cnt - 1
            else:
                print("No feasible Solution")
                best_velocity = [-0.1, 0.0]
                self.cnt = 1
        return best_velocity, best_trajectory
