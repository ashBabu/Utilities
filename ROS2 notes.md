### Create ROS2 packages

* `ros2 pkg create pkg_name --build-type ament_python --dependencies rclpy`  # for python

* `ros2 pkg create pkg_name --build-type ament_cmake --dependencies rclcpp`  # for C++

* `pip3 install setuptools==58.2.0`

* `colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja`

* `colcon build --symlink-install --packages-select sound_play`

* `ros2 launch urdf_tutorial display.launch.py model:=/home/ash/cr_ws/src/continuum_robot/urdf/snake_robot.xacro.urdf rvizconfig:=/home/ash/cr_ws/src/continuum_robot/rviz/sr_robot1.rviz`

* `sudo arp-scan -l`
* `ssh theia@192.168.0.33` 

### Robot upstart (deprecated, Use Docker based deployment)
* [Robot upstart](https://roboticsbackend.com/make-ros-launch-start-on-boot-with-robot_upstart/)
* `ros2 run robot_upstart install theia_obstacle_detection/launch/bringup.launch.py --job obstacle_alert --symlink`
* `sudo systemctl disable obstacle_alert`
* `sudo systemctl enable obstacle_alert.service`

### Some Gazebo command line utilities
* `ign gazebo empty.sdf` or `gz sim empty.sdf` **Note:** `ign` is for ignition Gazebo Fortress which is deprecated
* `ign/gz topic -e -t /gps/fix`  # echo (-e) topic (-t)
* `ign/gz topic --list`
* `ign topic -i --topic /camera/depth_image/points`  # info (-i)

### Publish to a topic in gazebo
* `ign topic -t "/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.5}"`

### Sam_bot
* `ros2 launch sam_bot_description display.launch.py`
* `ros2 launch slam_toolbox online_async_launch.py`
* `ros2 launch nav2_bringup navigation_launch.py params_file:=/home/ash/test_ws/src/navigation2_tutorials/sam_bot_description/config/nav2_params.yaml`

### Some ROS2 commands
* `ros2 run tf2_ros tf2_echo odom base_link`
* `ros2 topic echo /device/imu/data | grep -A 4 orientation`
### And and Not substitution 
* `rviz_node = Node(condition=IfCondition(AndSubstitution(NotSubstitution(run_headless), use_rviz))`
* `run_headless` needs to be `false`. `NotSubstitution` makes it `true`, `use_rviz` needs to be `true`. `AndSubtitution(true, true)` evalues to `true` to make `rviz_node` to run

* `ros2 service call /datum robot_localization/srv/SetDatum '{geo_pose: {position: {latitude: 51.507, longitude: 0.1276, altitude: 11}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'`  # for london

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

### Install cyclone DDS
* `sudo apt install -y ros-humble-rmw-cyclonedds-cpp`
* In `bashrc`, add `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`

**Note:**
The main issue was that IMU was not following ENU coordinate in Gazebo Fortress and has to be set manually.

```
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
```

### GPS logging via terminal
* `gpsd /dev/ttyACM0`
* `gpsmon`
* `sudo apt install gpsd-clients`
* `sudo apt install jq`
* `while true; do gpspipe -w -n 10 |   grep -m 1 lon | jq -r '(.lat|tostring) + " " + (.lon|tostring)'; done`

#### For Basalt
* `docker run -it -v /dev:/dev --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"   --device-cgroup-rule="c 81:* rmw"     --device-cgroup-rule="c 189:* rmw"     --device-cgroup-rule="c 234:* rmw"     --device-cgroup-rule="c 13:* rmw"     --device-cgroup-rule="c 510:* rmw"     --device-cgroup-rule="c 511:* rmw"     --device-cgroup-rule="c 166:* rmw" basalt_rs_cam`

**Note:** The accelerometer is an intertial sensor and it measures inertial force. So when the camera is idle, the accelerometer doesn’t measure G-force, but rather the force that resists to G. The resisting force aims up to the ceiling, and according to the coordinate System the positive Y-axis looks “down” and therefore the actual reading (the white line) is -9.8  (from https://github.com/IntelRealSense/librealsense/blob/master/doc/d435i.md)

### Getting USB devices to work in ADE
* `sudo usermod -aG dialout ash`
* `sudo chmod 666 /dev/ttyUSB0`
* `ls -l /dev | grep USB`

### Get inertiallabs IMU to work
* `rosrun inertiallabs_ins il_ins _ins_url:=serial:/dev/ttyUSB0:460800 _ins_output_format:=88`

### Full build and install
* `colcon build --merge-install --install-base install`

