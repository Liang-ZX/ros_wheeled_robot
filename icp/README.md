# ICP实验教程

### 运行指南

1. （在 home 目录下）新建工作空间 :

   > cd ~
   >
   > mkdir -p icp_ws/src

2. 将提供给大家地文件夹 course_agv_icp 放到 ~/icp_ws/src目录下

3. 编译工作空间

   > cd ~/icp_ws
   >
   > catkin_make
   >
   > source devel/setup.bash

4. 启动对应的 .launch 文件

   > roslaunch course_agv_icp icp.launch

5. rosbag运行

   > rosbag play xxx.bag

6. EKF

   > roslaunch course_agv_icp  localization.launch