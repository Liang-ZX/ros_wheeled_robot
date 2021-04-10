#!/usr/bin/env python2
import rospy
import tf
import numpy as np
import matplotlib.pyplot as plt
from Queue import Queue

class Planner:
    def __init__(self, map_data, map_info, grid_size, robot_radius):
        self.map_data = map_data
        self.map_info = map_info
        self.dilate_map = self.map_data.copy()
        self.grid_size = self.to_index(grid_size)
        self.robot_radius = self.to_index(robot_radius)
        self.offsets = [(1,0),(0,1),(-1,0),(0,-1),(1,1),(-1,1),(-1,-1),(1,-1)]
        self.obstacle_offsets = [(1,0),(0.707,0.707),(0,1),(-0.707,0.707),(-1,0),(-0.707,-0.707),(0,-1),(0.707,-0.707)]
    
    def update_planner(self, input_map):
        self.map_data = np.array(input_map.data).reshape((-1, input_map.info.height)).transpose()
        self.map_info = input_map.info
        self.dilate_map = self.map_data.copy()

    def planning(self, sx, sy, gx, gy):
        sx = self.to_index(sx - self.map_info.origin.position.x)
        sy = self.to_index(sy - self.map_info.origin.position.y)
        gx = self.to_index(gx - self.map_info.origin.position.x)
        gy = self.to_index(gy - self.map_info.origin.position.y)
        
        self.rx, self.ry = self.A_star(sx, sy, gx, gy)
        # self.rx, self.ry = self.bfs(sx, sy, gx, gy)
        
        if self.rx is not None:
            print(self.rx)
            print(self.ry)
        return self.rx, self.ry

    def A_star(self, sx, sy, gx, gy):
        visit = np.zeros((129,129)) # close list
        last = [[None for i in range(129)] for j in range (129)]
        found = False
        openlist = [[self.dist(sx, sy, gx, gy),0,(sx,sy)]]
        opendict = {(sx,sy): (self.dist(sx, sy, gx, gy),0)}
        cnt = 1
        while (cnt > 0):
            openlist.sort()
            [f,g,(now_x,now_y)] = openlist.pop(0)
            while visit[now_x, now_y]:
                [f,g,(now_x,now_y)] = openlist.pop(0)
            cnt = cnt - 1
            visit[now_x, now_y] = 1
            if (abs(now_x - gx)<self.grid_size) and (abs(now_y - gy)<self.grid_size):
                if not (now_x == gx and now_y == gy):
                    last[gx][gy] = (now_x, now_y)
                found = True
                break
            for (dx, dy) in self.offsets:
                dx *= self.grid_size
                dy *= self.grid_size
                if not visit[now_x+dx, now_y+dy]:
                    flag = True
                    if self.dilate_map[now_x+dx, now_y+dy] > 50: # self.map_data[now_x+dx, now_y+dy] > 50:
                        flag = False
                    elif self.dilate_map[now_x+dx, now_y+dy] > -50:
                        for scale in [1, 0.5]:
                            for (robot_x, robot_y) in self.obstacle_offsets:
                                robot_x = int(np.ceil(self.robot_radius * robot_x * scale))
                                robot_y = int(np.ceil(self.robot_radius * robot_y * scale))
                                if self.map_data[now_x + dx+robot_x, now_y+dy+robot_y] > 50:
                                    self.dilate_map[now_x+dx, now_y+dy] = 100 #update map
                                    flag = False
                                    break
                            if flag is False:
                                break
                    if flag is False:
                        visit[now_x+dx, now_y+dy] = 1
                    else:
                        self.dilate_map[now_x+dx, now_y+dy] = -100
                        if (now_x+dx, now_y+dy) in opendict:
                            (f_old, g_old) = opendict[(now_x+dx, now_y+dy)]
                            g_new = g + np.sqrt(dx**2+dy**2)
                            f_new = g_new + f_old - g_old
                            if f_new < f_old:
                                opendict[(now_x+dx, now_y+dy)] = (f_new, g_new)
                                last[now_x+dx][now_y+dy] = (now_x, now_y)
                                openlist.append([f_new, g_new, (now_x+dx, now_y+dy)])
                        else:
                            g_new = g + np.sqrt(dx**2+dy**2)
                            f_new = g_new + self.dist(now_x+dx, now_y+dy, gx, gy)
                            opendict[(now_x+dx, now_y+dy)] = (f_new, g_new)
                            last[now_x+dx][now_y+dy] = (now_x, now_y)
                            openlist.append([f_new, g_new, (now_x+dx, now_y+dy)])
                            cnt = cnt + 1
        if not found:
            print("The goal point is too close to obstacles!!!")
            return None, None
        else:
            print("The path is found.")
            rx, ry = [gx], [gy]
            tmp_x, tmp_y = gx, gy
            while last[tmp_x][tmp_y] is not None:
                (tmp_x, tmp_y) = last[tmp_x][tmp_y]
                rx.append(tmp_x)
                ry.append(tmp_y)
            return rx, ry                

    def bfs(self, sx, sy, gx, gy):
        q = Queue(maxsize=129*129)
        visit = np.zeros((129,129))
        last = [[None for i in range(129)] for j in range (129)]
        found = False
        visit[sx, sy] = 1
        q.put((sx, sy))
        while not q.empty():
            (now_x, now_y) = q.get()
            if (abs(now_x - gx)<self.grid_size) and (abs(now_y - gy)<self.grid_size):
                if not (now_x == gx and now_y == gy):
                    last[gx][gy] = (now_x, now_y)
                found = True
                break
            for (dx, dy) in self.offsets:
                dx *= self.grid_size
                dy *= self.grid_size
                if not visit[now_x+dx, now_y+dy]:
                    visit[now_x+dx, now_y+dy] = 1
                    flag = True
                    if self.dilate_map[now_x+dx, now_y+dy] > 50:
                        flag = False
                    elif self.dilate_map[now_x+dx, now_y+dy] > -50:
                        for scale in [1, 0.5]:
                            for (robot_x, robot_y) in self.obstacle_offsets:
                                robot_x = int(np.ceil(self.robot_radius * robot_x * scale))
                                robot_y = int(np.ceil(self.robot_radius * robot_y * scale))
                                if self.map_data[now_x + dx+robot_x, now_y+dy+robot_y] > 50:
                                    self.dilate_map[now_x+dx, now_y+dy] = 100 #update map
                                    flag = False
                                    break
                            if flag is False:
                                break
                    if flag:
                        self.dilate_map[now_x+dx, now_y+dy] = -100
                        last[now_x+dx][now_y+dy] = (now_x, now_y)
                        q.put((now_x+dx, now_y+dy))
        if not found:
            print("The goal point is too close to obstacles!!!")
            return None, None
        else:
            print("The path is found.")
            rx, ry = [gx], [gy]
            tmp_x, tmp_y = gx, gy
            while last[tmp_x][tmp_y] is not None:
                (tmp_x, tmp_y) = last[tmp_x][tmp_y]
                rx.append(tmp_x)
                ry.append(tmp_y)
            return rx, ry

    def to_index(self, x):
        return int(np.round(x/self.map_info.resolution))

    def dist(self, x1, y1, x2, y2):
        return np.sqrt((x2-x1)**2 + (y2-y1)**2)

