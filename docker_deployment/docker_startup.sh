#!/bin/bash

make_directory() {
    DIR_NAME=$1
    if [ ! -d "$DIR_NAME" ]; then
        echo "Directory $DIR_NAME does not exist. Creating it..."
        mkdir -p "$DIR_NAME" || { echo "Failed to create $DIR_NAME"; exit 1; }
    else
        echo "Directory $DIR_NAME already exists."
    fi
}

echo "Using HOME directory: $HOME"
make_directory "$HOME/saved_data"
cd "$HOME/saved_data"
CURRENTDATE=`date +"%Y-%m-%d"`
make_directory $CURRENTDATE

# source ros path
source $HOME/$GIT_REPO_NAME/ros2_ws/install/setup.bash

# Define the LiDAR IP
CETO_IP="10.43.0.19"
TELESTO_IP="10.42.0.18"

# Function to check if both LiDARs are reachable
check_lidars() {
    ping -c 1 $CETO_IP &> /dev/null
    local CETO_STATUS=$?
    
    ping -c 1 $TELESTO_IP &> /dev/null
    local TELESTO_STATUS=$?
    
    if [ $CETO_STATUS -eq 0 ] && [ $TELESTO_STATUS -eq 0 ]; then
        return 0  # Both LiDARs are reachable
    else
        return 1  # One or both LiDARs are not reachable
    fi
}

# Function to start ROS 2 launch and recording
start_ros_launch_and_record() {
    echo "Both LiDARs detected, launching ROS 2 node and recording topics."
    
    # Launch the ROS 2 launch file in the background
    ros2 launch bringup bringup.launch.py use_rtabmap_odom:=true & 
    LAUNCH_PID=$!  # Get the PID of the launch process

    cd $HOME/saved_data/$CURRENTDATE
    # Start recording bags every 3 seconds
    while check_lidars; do
        TIMESTAMP=$(date +%Y%m%d_%H%M%S)
        ros2 bag record /r2000_header /odom /odom_info /odometry/filtered /tf_static /ceto/scan /telesto/scan /imu/data_raw /imu/with_orientation /tf /imu/velocity /forward_camera/image_raw/compressed -o "rosbag_${TIMESTAMP}" --max-bag-duration 3 &> /dev/null 
        # ros2 bag record -a --compression-mode file --compression-format zstd -o "rosbag_${TIMESTAMP}" --max-bag-duration 3 &> /dev/null 
        RECORD_PID=$!

        # Wait for 3 seconds for the recording to complete before checking again
        sleep 3

        # Stop recording after each interval
        kill $RECORD_PID &> /dev/null
    done

    echo "Lost connection to one or both LiDARs, stopping ROS 2 node and recording."
    kill $LAUNCH_PID &> /dev/null  # Stop the launch file
}

# Main loop to continuously check LiDAR connection
while true; do
    if check_lidars; then
        start_ros_launch_and_record
    else
        echo "Waiting for both LiDARs to be reachable..."
        sleep 5
    fi
done