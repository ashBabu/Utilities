#!/bin/bash


# source ros path
source /opt/ros/humble/setup.bash
source /home/headlightai/headlight-slam/ros2_ws/install/setup.bash

ros2 launch hai_slam_bringup bringup.launch.py enable_ouster:=true lidar_mapping:=true &

sleep 60
while true; do
  ros2 service call /save_pcd lidar_mapping/srv/SavePCD "{leaf_size: 0.001, save_path: /home/headlightai/save_directory}"
  sleep 10  # Run every 10 seconds (adjust as needed)
done