## Integration of Moveit2 + Ign Gazebo

[Video](moveit_gazebo_integration)

### Steps
* Use moveit setup assistant to generate your `my_robot_moveit_config` package
* In `my_robot_moveit_config/config`, make a copy of `my_robot.ros2_control.xacro` and `my_robot.urdf.xacro`
* In the copy of `my_robot.ros2_control.xacro` ([example](src/XTa/continuum_robot_moveit_config/config/continuum_robot_gazebo.ros2_control.xacro)), replace the plugin under hardware with the following

                    <plugin>ign_ros2_control/IgnitionSystem</plugin>

* In the copy of `my_robot.urdf.xacro` ([example](src/XTa/continuum_robot_moveit_config/config/continuum_robot_gazebo.urdf.xacro)), make the following modifications as below

            <xacro:continuum_robot_ros2_control name="IgnitionSystem" initial_positions_file="$(arg initial_positions_file)"/>
            <gazebo>
                <plugin filename="ign_ros2_control-system" name="ign_ros2_control::IgnitionROS2ControlPlugin">
                    <parameters>$(find my_robot_moveit_config)/config/ros2_controllers.yaml</parameters>
                    <robot_param>robot_description</robot_param>
                    <robot_param_node>robot_state_publisher</robot_param_node>
                </plugin>
            </gazebo>

* Add the following launch files
        * [bringup.launch.py](src/XTa/continuum_robot_moveit_config/launch/bringup.launch.py)
        * [spawn_robot.launch.py](src/XTa/continuum_robot_moveit_config/launch/spawn_robot.launch.py)
        * [moveit_gazebo.launch.py](src/XTa/continuum_robot_moveit_config/launch/moveit_gazebo.launch.py)

* In `spawn_robot.launch.py`, the line

            load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'xta_arm_controller'],
        output='screen')
    
has to have the right controller name as generated in the [ros2_controllers.yaml](src/XTa/continuum_robot_moveit_config/config/ros2_controllers.yaml). Here `xta_arm_controller`.


* `cd ~/ros2_ws`
* `colcon build --symlink-install`
* `source install/setup.launch`

* `ros2 launch my_robot_moveit_config bringup.launch.py sim:=true`
* `ros2 launch my_robot_moveit_config moveit_gazebo.launch.py`