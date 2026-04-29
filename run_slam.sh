#!/bin/bash
ps aux | grep -E "rosmaster|roscore|roslaunch|rosout|semSLAM" | grep -v grep | awk '{print $2}' | xargs kill -9 2>/dev/null
sleep 10
cd /home/user/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch semgraph_slam semgraph_slam_mulran.launch visualize:=false
