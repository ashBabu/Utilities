import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    ld = LaunchDescription()

    pkg_directory = get_package_share_directory('continuum_robot')

    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]

        if "SDF_PATH" in os.environ:
            sdf_path = os.environ["SDF_PATH"]
            os.environ["SDF_PATH"] = sdf_path + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path

    # Launch configuration variables specific to simulation
    arg_x_pos = LaunchConfiguration('x_pos', default='0.0')
    arg_y_pos = LaunchConfiguration('y_pos', default='0.0')
    arg_z_pos = LaunchConfiguration('z_pos', default='1.5')
    arg_roll_ang = LaunchConfiguration('roll_ang', default='0.0')
    arg_pitch_ang = LaunchConfiguration('pitch_ang', default='0.0')
    arg_yaw_ang = LaunchConfiguration('yaw_ang', default='0.0')
    arg_use_gz_tf = LaunchConfiguration("use_gz_tf")
    arg_robot = LaunchConfiguration("robot", default='iris_with_ouster')  # iris_with_ouster, iris_with_d455
                                                                            # iris_with_cam_n_light, iris_with_blickfeld
    ld.add_action(DeclareLaunchArgument('x_pos', default_value='0.0',
                                        description='Specify x_position of the robot'))
    ld.add_action(DeclareLaunchArgument('y_pos', default_value='0.0',
                                        description='Specify y_position of the robot'))
    ld.add_action(DeclareLaunchArgument('z_pos', default_value='1.0',
                                        description='Specify z_position of the robot'))
    ld.add_action(DeclareLaunchArgument('roll_ang', default_value='0.0',
                                        description='Specify initial roll angle of the robot'))
    ld.add_action(DeclareLaunchArgument('pitch_ang', default_value='0.0',
                                        description='Specify initial pitch angle of the robot'))
    ld.add_action(DeclareLaunchArgument('yaw_ang', default_value='0.0',
                                        description='Specify initial yaw angle of the robot'))
    ld.add_action(DeclareLaunchArgument("use_gz_tf", default_value="true", description="Use Gazebo TF"))
    ld.add_action(DeclareLaunchArgument('robot', default_value='iris_with_ouster'))

    robot_description = ParameterValue(
        Command(['xacro ', str(get_package_share_path('continuum_robot') / 'urdf/snake_robot.urdf')]),
        value_type=str)
    
    node_robot_state_publisher= Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        #parameters=[{'use_sim_time': use_sim_time}],
        # arguments=[urdf_file]
        parameters=[{'robot_description': robot_description}]
    )
    ld.add_action(node_robot_state_publisher)

    node_gz_ros_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', "XTa",
            # '-file', arg_robot,
            '-topic', 'robot_description',
            '-x', arg_x_pos,
            '-y', arg_y_pos,
            '-z', arg_z_pos,
            '-R', arg_roll_ang,
            '-P', arg_pitch_ang,
            '-Y', arg_yaw_ang
        ],
        output='screen',
    )
    ld.add_action(node_gz_ros_spawner)

    # include_robot_state_publisher = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_directory, 'launch', 'robot_state_publisher.launch.py')
    #     ),
    #     launch_arguments={
    #         'robot': arg_robot
    #     }.items()
    # )
    # ld.add_action(include_robot_state_publisher)

    bridge_params = os.path.join(pkg_directory, 'config', 'bridge.yaml')

    node_gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {
                "config_file": bridge_params,
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )
    ld.add_action(node_gz_ros_bridge)

    node_gz_ros_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['rgbd_camera/image', 'rgbg_camera/depth_image', '/flexx2'],
        output='screen',
    )
    ld.add_action(node_gz_ros_image_bridge)

    # Relay - use instead of transform when Gazebo is only publishing odom -> base_link
    node_topic_tools_tf = Node(
        package="topic_tools",
        executable="relay",
        arguments=[
            "/gz/tf",
            "/tf",
        ],
        output="screen",
        respawn=False,
        condition=IfCondition(arg_use_gz_tf),
    )
    include_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=node_gz_ros_bridge,
            on_start=[
                node_topic_tools_tf
            ]
        )
    )
    # ld.add_action(include_event_handler)  # for gazebo odometry

    return ld