# ROS-wheeled-robot

Author: Zhixuan Liang, Fanghao Wang, Zhenze Jiang

Tutorial: [Tutorial](document/)

ROS Official Doc: [ROS](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

Quick Start

1. Path planning with BFS

```bash
# build source
catkin build
source devel/setup.bash

# create roscore in one console
roscore 

# open another console
roslaunch course_agv_nav nav.launch
```

![bfs_img](document/images/results/bfs.png)
