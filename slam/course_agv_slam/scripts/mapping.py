import math
import numpy as np
import matplotlib.pyplot as plt
from collections import deque


class Mapping():
    def __init__(self,xw,yw,xyreso):
        self.width_x = xw*xyreso
        self.width_y = yw*xyreso
        self.xyreso = xyreso
        self.xw = xw
        self.yw = yw
        self.pmap = np.ones((self.xw, self.yw))*0.3 # default 0.5 -- [[0.5 for i in range(yw)] for i in range(xw)] 
        self.pmap[0,:] = 1
        self.pmap[-1,:] = 1
        self.pmap[:,-1] = 1
        self.pmap[:,0] = 1
        self.minx = -self.width_x/2.0
        self.maxx =  self.width_x/2.0
        self.miny = -self.width_y/2.0
        self.maxy =  self.width_y/2.0
        self.sensor_reflex_p = 0.9
        self.sensor_pass_p = 0.1
        self.update_reflex = np.log(self.sensor_reflex_p/(1-self.sensor_reflex_p))
        self.update_pass = np.log(self.sensor_pass_p / (1-self.sensor_pass_p))
        pass
    
    
    # ox, oy is the coordinate of laser points in the world frame
    # center_x, center_y is the coordinate of the robot position in the world frame
    # TODO: update the value of the grid map, 0 means free space, 1 means obstacle, value between [0, 1] means unexplored areas

    def update(self, ox, oy, center_x, center_y):
        xyreso = self.xyreso
        minx,miny,maxx,maxy,pmap = self.minx,self.miny,self.maxx,self.maxy,self.pmap
        centix, centiy = (center_x-minx) * 1.0 / xyreso, (center_y-miny) * 1.0 / xyreso
        #step 1.  whether the robot is out of range
        if self.outOfRange(centix,centiy):
            pmap = np.clip(pmap,0,1)
            return pmap

        # occupancy grid computed with ray casting
        for (x, y) in zip(ox, oy):
            # transform the coordinate into the frame of the map: (ix, iy)
            ix, iy = (x-minx)*1.0/xyreso, (y-miny)*1.0/xyreso
            if self.outOfRange(ix,iy):
                continue

            # ray_casting
            laser_beams = self.ray_casting((centix, centiy), (ix, iy)) # line form the lidar to the point
        
        for pos in laser_beams[:-1]:
            tmp_p = pmap[pos[0], pos[1]]
            tmp_l =  np.log(tmp_p / (1-tmp_p)) + self.update_pass
            pmap[pos[0], pos[1]] = 1 - 1.0 / (1+np.exp(tmp_l))

        pos = laser_beams[-1]
        tmp_p = pmap[pos[0], pos[1]]
        tmp_l =  np.log(tmp_p / (1-tmp_p)) + self.update_reflex
        pmap[pos[0], pos[1]] = 1 - 1.0 / (1+np.exp(tmp_l))

        pmap = np.clip(pmap,0,1)

        return pmap
        
    def outOfRange(self,x,y):
        return x > self.pmap.shape[0] or x < 0 or y < 0 or y > self.pmap.shape[1]

    #implement your algorithm
    def ray_casting(self,start, end):
        """
        >>> points1 = ray_casting((4, 4), (6, 10))
        >>> print(points1)
        np.array([[4,4], [4,5], [5,6], [5,7], [5,8], [6,9], [6,10]])
        """
        points = [np.array(start)]
        x1, y1 = start
        x2, y2 = end
        ## Attention: Bresenham Line Drawing Algorithm
        dx, dy = np.fabs(x2-x1), np.fabs(y2-y1)
        do_swap = False
        if dx < dy:
            do_swap = True  # |gradient| > 1
            x1, y1 = y1, x1
            x2, y2 = y2, x2
            dx, dy = dy, dx
        ix = 1 if x2 > x1 else -1
        iy = 1 if y2 > y1 else -1
        tx, ty = x1, y1
        P = 2 * dy - dx
        while tx != x2:
            if P > 0:
                ty = ty + iy
                P = P + 2 * (dy - dx)
            else:
                P = P + 2 * dy
            if not do_swap:
                points.append(np.array([tx, ty]))
            else:
                points.append(np.array([ty, tx]))
            tx = tx + ix
        
        points = np.array(points)
        return points

