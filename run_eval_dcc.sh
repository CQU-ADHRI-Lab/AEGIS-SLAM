#!/bin/bash
cd /home/user/catkin_ws/src/SG-SLAM/eval
python3 traj_eval.py --gt_file /data/03/dcc01.txt --pred_file /home/user/catkin_ws/src/SG-SLAM/ros/save/mulran_slam_poses_03.txt > /home/user/catkin_ws/dcc01_eval.txt 2>&1
