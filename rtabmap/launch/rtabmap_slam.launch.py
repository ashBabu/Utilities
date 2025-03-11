import os
from datetime import datetime
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    ld = LaunchDescription()
    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    database_path = os.path.expanduser(f'~/.rtabmap/slam_{timestamp}.db')

    arg_base_frame = LaunchConfiguration('base_frame', default='base_link')
    arg_imu_topic = LaunchConfiguration(
        'imu_topic', default='/camera/imu/data')
    arg_rgb_img_topic = LaunchConfiguration(
        'rgb_img_topic', default='/camera/color/image_raw')
    arg_rgb_cam_info_topic = LaunchConfiguration(
        'rgb_cam_info_topic', default='/camera/color/camera_info')
    arg_depth_img_topic = LaunchConfiguration(
        'depth_img_topic', default='/camera/aligned_depth_to_color/image_raw')
    arg_lidar_scan_topic = LaunchConfiguration('lidar_scan_topic', default='/okdo/scan')

    # Tune rtabmap parameters:
    # http://wiki.ros.org/rtabmap_ros/Tutorials/Advanced%20Parameter%20Tuning#Visual_Odometry
    # ICP odometry wont run with realsense D435i :
    # https://github.com/introlab/rtabmap_ros/issues/1061#issuecomment-1806942969
    # Rtabmap Odometry examples:
    # https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_examples/launch
    parameters = [
        {'rtabmap_args': '--delete_db_on_start'},
        {'database_path': database_path},
        {'subscribe_depth': True},
        {'subscribe_scan': True},
        {'subscribe_odom_info': True},
        {'approx_sync': True},
        {"frame_id": arg_base_frame},
        {"odom_frame_id": "odom"},
        {"queue_size": 50},
        {"qos": 2},
        {"qos_imu": 2},
        # {"publish_tf": True},
    ]

    remappings = [
        ('imu', arg_imu_topic),
        ('rgb/image', arg_rgb_img_topic),
        ('rgb/camera_info', arg_rgb_cam_info_topic),
        ('depth/image', arg_depth_img_topic),
        ('scan', arg_lidar_scan_topic),
        ('odom', '/odometry/filtered'),
    ]

    node_rtabmap_slam = Node(
        package='rtabmap_slam', 
        executable='rtabmap', 
        output='screen',
        namespace='/rtabamp',
        parameters=parameters,
        remappings=remappings,
        arguments=['-d']
    )
    ld.add_action(node_rtabmap_slam)

    return ld