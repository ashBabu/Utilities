#!/usr/bin/env python3
import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)


def generate_launch_description():

    ld = LaunchDescription()

    pkg_directory = get_package_share_directory('continuum_robot')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    default_rviz_file = os.path.join(pkg_directory, 'rviz', 'bringup.rviz')
    default_world_name = 'simple_world'  # y_tunnel, indian_tunnel
    default_world_path = os.path.join(pkg_directory, 'worlds')

    arg_x_pos = LaunchConfiguration('x_pos', default='0.0')
    arg_y_pos = LaunchConfiguration('y_pos', default='0.0')
    arg_z_pos = LaunchConfiguration('z_pos', default='1.0')
    arg_rviz = LaunchConfiguration('rviz')
    arg_rviz_file = LaunchConfiguration('rviz_file')
    arg_world_name = LaunchConfiguration('world_name', default=default_world_name)
    arg_world = LaunchConfiguration('world', default=default_world_path)

    # Declare the launch arguments
    ld.add_action(DeclareLaunchArgument('x_pos', default_value='0.0',
                                        description='Specify x_position of the robot'))
    ld.add_action(DeclareLaunchArgument('y_pos', default_value='0.0',
                                        description='Specify y_position of the robot'))
    ld.add_action(DeclareLaunchArgument('z_pos', default_value='1.0',
                                        description='Specify z_position of the robot'))
    ld.add_action(DeclareLaunchArgument('rviz', default_value='false',
                                        description='whether to launch rviz or not'))
    ld.add_action(DeclareLaunchArgument('rviz_file', default_value=default_rviz_file))
    ld.add_action(DeclareLaunchArgument('world_name', default_value=default_world_name))
    ld.add_action(DeclareLaunchArgument('world', default_value=[arg_world, '/', arg_world_name, '.world']))

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_directory, 'models'))
    ld.add_action(set_env_vars_resources)

    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]

        if "SDF_PATH" in os.environ:
            sdf_path = os.environ["SDF_PATH"]
            os.environ["SDF_PATH"] = sdf_path + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path

    include_gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -v4 ', arg_world]}.items()
    )
    ld.add_action(include_gzserver)

    include_spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_directory, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'x_pos': arg_x_pos,
            'y_pos': arg_y_pos,
            'z_pos': arg_z_pos
        }.items()
    )
    ld.add_action(include_spawn_robot)

    # RViz.
    node_rviz = Node(
        condition=IfCondition(arg_rviz),
        package="rviz2",
        executable="rviz2",
        name='rviz2',
        arguments=[
            "-d",
            arg_rviz_file
        ],
    )
    ld.add_action(node_rviz)

    return ld