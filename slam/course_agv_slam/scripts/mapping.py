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
        pass
    
    
    # ox, oy is the coordinate of laser points in the world frame
    # center_x, center_y is the coordinate of the robot position in the world frame
    # TODO: update the value of the grid map, 0 means free space, 1 means obstacle, value between [0, 1] means unexplored areas

    def update(self, ox, oy, center_x, center_y):
        xyreso = self.xyreso
        minx,miny,maxx,maxy,pmap = self.minx,self.miny,self.maxx,self.maxy,self.pmap
        
        #step 1.  whether the robot is out of range
        if self.outOfRange(centix,centiy):
            pmap = np.clip(pmap,0,1)
            return pmap

        # occupancy grid computed with ray casting
        for (x, y) in zip(ox, oy):

            # transform the coordinate into the frame of the map: (ix, iy)
           
            if self.outOfRange(ix,iy):
                continue

            # ray_casting
            laser_beams = self.ray_casting((centix, centiy), (ix, iy)) # line form the lidar to the point

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
        
        return points

