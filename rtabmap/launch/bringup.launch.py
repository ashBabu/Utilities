#!/usr/bin/env python3
import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, NotSubstitution, AndSubstitution, OrSubstitution


def generate_launch_description():

    ld = LaunchDescription()
    package_dir = get_package_share_directory('bringup')
    package_perception = get_package_share_directory('perception')
    default_rviz_file = os.path.join(package_dir, 'rviz', 'viz.rviz')

    arg_sim = LaunchConfiguration('sim')
    arg_rviz2 = LaunchConfiguration('rviz2')
    arg_rviz_file = LaunchConfiguration('rviz_file')
    arg_rtab_rgbd_odom = LaunchConfiguration('rtab_rgbd_odom')
    arg_realsense = OrSubstitution(AndSubstitution(
        NotSubstitution(arg_sim), arg_rtab_rgbd_odom), LaunchConfiguration('realsense'))
    arg_okdo = LaunchConfiguration('okdo')
    arg_rtab_icp_odom = LaunchConfiguration('rtab_icp_odom')
    arg_base_frame = LaunchConfiguration('base_frame')
    arg_lidar_scan_topic = LaunchConfiguration('lidar_scan_topic')
    arg_lidar_odom_topic = LaunchConfiguration('lidar_odom_topic')
    arg_imu_topic_without_orientation = LaunchConfiguration(
        'imu_topic_unfiltered', default='/camera/imu')
    arg_imu_topic_with_orientation = LaunchConfiguration(
        'imu_topic_filtered', default='/camera/imu/data')
    arg_rgbd_odom_pub_topic = LaunchConfiguration('rgbd_odom_pub_topic')
    arg_rgb_img_topic = LaunchConfiguration('rgb_img_topic')
    arg_rgb_cam_info_topic = LaunchConfiguration('rgb_cam_info_topic')
    arg_depth_img_topic = LaunchConfiguration('depth_img_topic')
    arg_rtab_slam = LaunchConfiguration('rtab_slam')

    ld.add_action(DeclareLaunchArgument('sim', default_value='false',
                                        description='whether to launch real or simulated robot'))
    ld.add_action(DeclareLaunchArgument(
        'rviz2', default_value='false', description='open rviz or not'))
    ld.add_action(DeclareLaunchArgument(
        'rviz_file', default_value=default_rviz_file))
    ld.add_action(DeclareLaunchArgument('realsense',
                  default_value='false', description='start rs cam or not'))
    ld.add_action(DeclareLaunchArgument('okdo', default_value='false',
                                        description='whether to start okdo lidar'))
    ld.add_action(DeclareLaunchArgument(
        'rtab_icp_odom', default_value='false', description='start icp odometry or not'))
    ld.add_action(DeclareLaunchArgument(
        'base_frame', default_value='base_link'))
    ld.add_action(DeclareLaunchArgument(
        'lidar_scan_topic', default_value='/okdo/scan'))
    ld.add_action(DeclareLaunchArgument(
        'lidar_odom_topic', default_value='rtabmap/okdo/odom', description='topic to publish icp odometry'))
    ld.add_action(DeclareLaunchArgument('rtab_rgbd_odom',
                  default_value='false', description='start rgbd odometry or not'))
    ld.add_action(DeclareLaunchArgument('rgbd_odom_pub_topic',
                  default_value='/rtabmap/realsense/rgbd/odom', description='topic to publish rgbd odometry'))
    ld.add_action(DeclareLaunchArgument('rgb_img_topic',
                  default_value='/camera/color/image_raw', description='rgb image topic'))
    ld.add_action(DeclareLaunchArgument('rgb_cam_info_topic',
                  default_value='/camera/color/camera_info', description='rgb cam info topic'))
    ld.add_action(DeclareLaunchArgument('depth_img_topic',
                  default_value='/camera/aligned_depth_to_color/image_raw', description='depth image topic'))
    ld.add_action(DeclareLaunchArgument('rtab_slam',
                  default_value='false', description='start slam node'))

    # If simulation, launch robot in gazebo
    include_robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('description'),
                         'launch', 'bringup.launch.py')
        ),
        condition=IfCondition(arg_sim),
    )
    ld.add_action(include_robot_description)

    # Launch ldlidar
    include_okdo_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ldlidar_stl_ros2'),
                         'launch', 'ld06.launch.py')
        ),
        condition=IfCondition(arg_okdo)
    )
    ld.add_action(include_okdo_lidar)

    """
    Rtabmap provides ICP odometry
    https://wiki.ros.org/rtabmap_odom#icp_odometry
    https://github.com/introlab/rtabmap_ros/blob/humble-devel/rtabmap_launch/launch/rtabmap.launch.py
    """
    include_rtab_icp_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_perception, 'launch', 'rtabmap_icp_odom.launch.py')),
        launch_arguments={
            "scan_topic": arg_lidar_scan_topic,
            "odom_topic": arg_lidar_odom_topic,
            "base_frame": arg_base_frame,
            "need_tf": "False"
        }.items(),
        condition=IfCondition(arg_rtab_icp_odom)
    )
    ld.add_action(include_rtab_icp_odom)

    # Rtabmap RGBD odometry
    include_rs_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_perception, 'launch', 'realsense_cam.launch.py')),
        condition=IfCondition(arg_realsense)
    )
    ld.add_action(include_rs_cam)

    include_imu_filter_madgwick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_perception, 'launch', 'imu_filter_madgwick.launch.py')),
        condition=IfCondition(arg_realsense),
        launch_arguments={
            "imu_topic_to_filter": arg_imu_topic_without_orientation,
            "imu_topic_filtered": arg_imu_topic_with_orientation,
        }.items(),
    )
    ld.add_action(include_imu_filter_madgwick)

    include_rtab_rgbd_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_perception, 'launch', 'rtabmap_rgbd_odom.launch.py')),
        condition=IfCondition(arg_rtab_rgbd_odom),
        launch_arguments={
            "rgb_img_topic": arg_rgb_img_topic,
            "rgb_cam_info_topic": arg_rgb_cam_info_topic,
            "depth_img_topic": arg_depth_img_topic,
            "odom_pub_topic": arg_rgbd_odom_pub_topic,
            "imu_topic": arg_imu_topic_with_orientation,
            "base_frame": arg_base_frame,
            "need_tf": "False",
        }.items(),
    )
    ld.add_action(include_rtab_rgbd_odom)

    include_rtab_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_perception, 'launch', 'rtabmap_slam.launch.py')),
        condition=IfCondition(arg_rtab_slam),
        launch_arguments={
            "rgb_img_topic": arg_rgb_img_topic,
            "rgb_cam_info_topic": arg_rgb_cam_info_topic,
            "depth_img_topic": arg_depth_img_topic,
            "imu_topic": arg_imu_topic_with_orientation,
            "base_frame": arg_base_frame,
            "lidar_scan_topic": arg_lidar_scan_topic,
        }.items(),
    )
    ld.add_action(include_rtab_slam)

    # Robot localization
    include_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('navigation'),
                         'launch', 'localization.launch.py')
        ),
        launch_arguments={
            "use_sim_time": arg_sim
        }.items(),
        condition=IfCondition(OrSubstitution(
            arg_rtab_icp_odom, arg_rtab_rgbd_odom))
    )
    ld.add_action(include_localization)

    include_baselink_cam_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('description'),
                         'launch', 'static_transform_pub.launch.py')
        ),
        condition=IfCondition(AndSubstitution(
            NotSubstitution(arg_sim), arg_realsense))
        # should evaluate to true when sim=false and realsense=true
    )
    ld.add_action(include_baselink_cam_tf)

    # RViz.
    node_rviz = Node(
        condition=IfCondition(arg_rviz2),
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