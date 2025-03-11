import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    param_config = os.path.join(get_package_share_directory(
        'perception'), 'config', 'imu_filter.yaml')
    arg_imu_topic_unfiltered = LaunchConfiguration("imu_topic_unfiltered")
    arg_imu_topic_filtered = LaunchConfiguration("imu_topic_filtered")

    ld.add_action(DeclareLaunchArgument(
        'imu_topic_unfiltered', default_value='/camera/imu'))
    ld.add_action(DeclareLaunchArgument(
        'imu_topic_filtered', default_value='/camera/imu/data'))

    node_imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[param_config],
        remappings=[
            ('/imu/data_raw', arg_imu_topic_unfiltered),
            ('/imu/data', arg_imu_topic_filtered)
        ]
    )
    ld.add_action(node_imu_filter)

    return ld