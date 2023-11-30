ros2 pkg create pkg_name --build-type ament_python --dependencies rclpy  # for python
ros2 pkg create pkg_name --build-type ament_cmake --dependencies rclcpp

pip3 install setuptools==58.2.0
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
colcon build --symlink-install --packages-select sound_play


ADE
ros-humble-rtabmap
ros-humble-realsense2-description
ros-humble-realsense2-camera

docker build -t ghcr.io/theia-robotics/ade-image ade-image

ros2 run realsense2_camera realsense2_camera_node

ros2 run rviz2 rviz2


sudo apt-get install -y  libsdl2-dev libsdl2-mixer-dev libsdl2-mixer-2.0-0 

        // TODO: find the best QOS (KeepLast 10 or less?)

Copyright statement: Nobody is allowed to copy, use or distribute this code other than Theia Guidance System employees



### ADE usage
`cd theia_home/theia`
`./start=ade`
`sudo apt update`
`cd ../ros2_ws`
`rosdep install --from-paths src --ignore-src -r -y`
`colcon build --symlink-install`
`source install/setup.bash`
`ros2 launch theia_obstacle_detection bringup.launch.xml`

ros2 launch urdf_tutorial display.launch.py model:=/home/ash/cr_ws/src/continuum_robot/urdf/snake_robot.xacro.urdf rvizconfig:=/home/ash/cr_ws/src/continuum_robot/rviz/sr_robot1.rviz


* `sudo arp-scan -l`
* `ssh theia@192.168.0.33` 

gazebo worlds/simple_city_static.world

https://roboticsbackend.com/make-ros-launch-start-on-boot-with-robot_upstart/
ros2 run robot_upstart install theia_obstacle_detection/launch/bringup.launch.py --job obstacle_alert --symlink
sudo systemctl disable obstacle_alert
sudo systemctl enable obstacle_alert.service



ros2 launch theia_gazebo bringup.launch.py
ign topic -e -t /gps/fix  # echo (-e) topic (-t)
ign topic --list
ign topic -i --topic /camera/depth_image/points  # info (-i)


theia_navigation
    robot_state_publisher
    robot_localization_node
    global planner and controllers


ign topic -t "/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.5}"


### Sam_bot
* `ros2 launch sam_bot_description display.launch.py`
* `ros2 launch slam_toolbox online_async_launch.py`
* `ros2 launch nav2_bringup navigation_launch.py params_file:=/home/ash/test_ws/src/navigation2_tutorials/sam_bot_description/config/nav2_params.yaml`


ros2 run tf2_ros tf2_echo odom base_link
ros2 launch theia_bringup bringup.launch.py sim:=true rviz:=true use_command_velocity:=true headless:=false use_navigation:=true use_ground_truth:=true 
sudo apt update && sudo apt install ros-humble-pointcloud-to-laserscan

rviz_node = Node(condition=IfCondition(AndSubstitution(NotSubstitution(run_headless), use_rviz)),


ros2 service call /datum robot_localization/srv/SetDatum '{geo_pose: {position: {latitude: 51.507, longitude: 0.1276, altitude: 11}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'  # for london


ros2 launch theia_obstacle_detection test.launch.py  use_rviz:=true
ros2 bag play rosbag2_2023_09_26-13_40_13/ --loop

### Working ROS2 navigation
* `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`
* `ros2 launch sam_bot_description online_async_launch.py`  ## might need to source the environment `source test_ws/install/setup.bash`
* `ros2 launch sam_bot_description test_nav2.launch.py`

### Explanation
The launch file `turtlebot3_world.launch.py` would provide all the transforms from `odom` --> `base_footprint` --> rest of the robot. The `odom` --> `base_footprint` is provided by the differential drive plugin. `online_async_launch.py` provides the transform `map` --> `odom` and also publishes to the `/map` topic. Now the map or the occupancy grid to get started is available. This is required for the planner (global planner in ROS1) to plan an obstacle free path from point A to point B. Once an obstacle free path is planned, then the controller (local planner in ROS1) takes charge and tries to control the robot to follow the global plan keeping in mind of the dynamic or any new static obstacles in the path.


### Kind of working theia version
* `ros2 launch theia_bringup test_bringup.launch.py sim:=true rviz:=true use_command_velocity:=true headless:=false use_ground_truth:=true`
* `ros2 launch theia_navigation slam_online_async.launch.py`
* `ros2 launch theia_navigation test_nav2.launch.py`


ros2 topic echo /device/imu/data | grep -A 4 orientation

sudo apt install -y ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp


ros2 launch theia_gazebo bringup.launch.py  headless:=false
ros2 launch theia_navigation robot_localization.launch.py
The main issue was that IMU was not following ENU coordinate in Gazebo Fortress and has to be set manually.


<gazebo>
    <plugin filename="ignition-gazebo-velocity-control-system"
            name="ignition::gazebo::systems::VelocityControl">
      <topic>cmd_vel</topic>
    </plugin>
    <plugin filename="ignition-gazebo-odometry-publisher-system"
            name="ignition::gazebo::systems::OdometryPublisher">
      <odom_frame>odom</odom_frame>
      <dimensions>3</dimensions>
      <odom_topic>odom</odom_topic>
      <odom_covariance_topic>odom</odom_covariance_topic>
      <tf_topic>tf</tf_topic>
      <robot_base_frame>base_link</robot_base_frame>
      <odom_publish_frequency>100</odom_publish_frequency>
    </plugin>

  </gazebo>

ros2 bag record /odometry/gps /odometry/filtered /tf /tf_static /fix /camera/imu /imu/data /camera/accel/imu_info /odom /camera/gyro/imu_info /extended_fix

ros2 launch gpsd_client gpsd_client-launch.py

mapviz_key: AqwnKvrPZnjI7CQFO0wXUmKjlmtks-qaI_IgMSgdJWtMoCFaR4zXkkghMGqgtpgC


### GPS logging via terminal
`gpsd /dev/ttyACM0`
`gpsmon`
sudo apt install gpsd-clients
sudo apt install jq
while true; do gpspipe -w -n 10 |   grep -m 1 lon | jq -r '(.lat|tostring) + " " + (.lon|tostring)'; done


#### For Basalt
docker run -it -v /dev:/dev --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"   --device-cgroup-rule="c 81:* rmw"     --device-cgroup-rule="c 189:* rmw"     --device-cgroup-rule="c 234:* rmw"     --device-cgroup-rule="c 13:* rmw"     --device-cgroup-rule="c 510:* rmw"     --device-cgroup-rule="c 511:* rmw"     --device-cgroup-rule="c 166:* rmw" basalt_rs_cam


Note: The accelerometer is an intertial sensor and it measures inertial force. So when the camera is idle, the accelerometer doesn’t measure G-force, but rather the force that resists to G. The resisting force aims up to the ceiling, and according to the coordinate System the positive Y-axis looks “down” and therefore the actual reading (the white line) is -9.8  (from https://github.com/IntelRealSense/librealsense/blob/master/doc/d435i.md)


### ADE error
docker run -h ade --detach --name ade --env COLORFGBG --env COLORTERM --env EMAIL --env GIT_AUTHOR_EMAIL --env GIT_AUTHOR_NAME --env GIT_COMMITTER_EMAIL --env GIT_COMMITTER_NAME --env SSH_AUTH_SOCK --env TERM --env TIMEZONE=Europe/London --env USER=ash --env GROUP=ash --env USER_ID=1000 --env GROUP_ID=1000 -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/ash/Ash/basalt_home:/home/ash --env ADE_CLI_VERSION=4.4.0 --env ADE_HOME_HOSTPATH=/home/ash/Ash/basalt_home --label ade_version=4.4.0 --label ade_base_image --env DISPLAY --env VIDEO_GROUP_ID=44 -v /dev/dri:/dev/dri -v /home/ash/.ssh:/home/ash/.ssh -v /run/user/1000/keyring/ssh:/run/user/1000/keyring/ssh --volumes-from ade_registry.gitlab.com_apexai_ade-vscode_1.71.2:ro --label 'ade_volumes_from=["ade_registry.gitlab.com_apexai_ade-vscode_1.71.2"]' '--device-cgroup-rule=c 81:* rmw' '--device-cgroup-rule=c 189:* rmw' '--device-cgroup-rule=c 234:* rmw' '--device-cgroup-rule=c 13:* rmw' '--device-cgroup-rule=c 510:* rmw' '--device-cgroup-rule=c 511:* rmw' '--device-cgroup-rule=c 166:* rmw' -v /dev:/dev '#-e' NVIDIA_DRIVER_CAPABILITIES=compute,video,utility --env ADE_IMAGE_BASALT_ADE_IMAGE_FQN=basalt_ade_image:latest --env ADE_IMAGE_BASALT_ADE_IMAGE_COMMIT_SHA= --env ADE_IMAGE_BASALT_ADE_IMAGE_COMMIT_TAG= --env ADE_IMAGE_ADE_VSCODE_FQN=registry.gitlab.com/apexai/ade-vscode:1.71.2 --env ADE_IMAGE_ADE_VSCODE_COMMIT_SHA=9601b5676f40a95932847b19a3820ad31871e91b --env ADE_IMAGE_ADE_VSCODE_COMMIT_TAG=1.71.2 --label 'ade_images=[{"fqn": "basalt_ade_image:latest", "commit_sha": "", "commit_tag": ""}, {"fqn": "registry.gitlab.com/apexai/ade-vscode:1.71.2", "commit_sha": "9601b5676f40a95932847b19a3820ad31871e91b", "commit_tag": "1.71.2"}]' basalt_ade_image:latest


