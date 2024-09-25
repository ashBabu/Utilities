#!/bin/bash


# source ros path
source /opt/ros/humble/setup.bash
source /home/headlightai/headlight-slam/ros2_ws/install/setup.bash

# Define the LiDAR IP (replace with your Ouster's actual IP)
LIDAR_IP="10.44.0.110"
LIDAR_DETECTED=false

# Check if the LiDAR is reachable by pinging its IP address
while [ "$LIDAR_DETECTED" = false ]; do
    if ping -c 1 $LIDAR_IP &> /dev/null; then
        echo "LiDAR detected, starting ROS 2 launch..."
        LIDAR_DETECTED=true
        sleep 10
        ros2 launch hai_slam_bringup bringup.launch.py enable_ouster:=true lidar_mapping:=true &
    else
        echo "LiDAR not detected, retrying in 5 seconds..."
        sleep 5
    fi
done

sleep 60
cd $HOME
# if saved_pcds dir is not existant, create and enter it.
if [ ! -d "saved_pcds" ]; then
	mkdir saved_pcds
	echo "saved_pcds directory doesnot exist. Creating it in HOME/rosbags"
fi
cd saved_pcds


# create a dir with current date and enter it if it doesnot already exist
CURRENTDATE=`date +"%Y-%m-%d"`
if [ ! -d ${CURRENTDATE} ]; then
	mkdir ${CURRENTDATE}
	echo directory created
fi
cd ${CURRENTDATE}

while true; do
  ros2 service call /save_pcd lidar_mapping/srv/SavePCD "{leaf_size: 0.001, save_path: $HOME/saved_pcds/${CURRENTDATE}/}"
  sleep 10  # Run every 10 seconds (adjust as needed)
done