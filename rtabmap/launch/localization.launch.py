import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    ld = LaunchDescription()

    robot_localization_dir = get_package_share_directory('navigation')
    parameters_file_path = os.path.join(robot_localization_dir, 'config', 'localization.yaml')

    arg_use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))
    ld.add_action(DeclareLaunchArgument('output_final_position', default_value='false'))
    ld.add_action(DeclareLaunchArgument('output_location', default_value='~/dual_ekf_navsat_example_debug.txt'))

    # === Localization node ===

    # ref1: http://docs.ros.org/en/noetic/api/robot_localization/html/navsat_transform_node.html
    # ref2: http://docs.ros.org/en/noetic/api/robot_localization/html/state_estimation_nodes.html#ekf-localization-node
    # ref3: https://navigation.ros.org/tutorials/docs/navigation2_with_gps.html

    node_ekf_filter_map = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[parameters_file_path, {"use_sim_time": arg_use_sim_time}],
    )
    ld.add_action(node_ekf_filter_map)

    return ld