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
        # {'subscribe_odom_info': True},
        # {'approx_sync': True},
        {"frame_id": arg_base_frame},
        # {"odom_frame_id": "odom"},
        # {"queue_size": 50},
        # {"qos": 2},
        # {"qos_imu": 2},
        # {"qos_imu": 2},
        {"Reg/Strategy": "1"},                         # "0=Vis, 1=Icp, 2=VisIcp"
        {"Reg/Force3DoF": "True"},                    
        # {'Rtabmap/DetectionRate': "1"},              # Detection rate (Hz). RTAB-Map will filter input images to satisfy this rate.
        # {'Rtabmap/ImagesAlreadyRectified': "False"},   # By default RTAB-Map assumes that received images are rectified. If they are not, they can be rectified by RTAB-Map if this parameter is false.
        # {'Rtabmap/LoopThr': "0.11"},                   # Loop closing threshold.
        {'Odom/Strategy': "0"},                        # [0=Frame-to-Map (F2M) 1=Frame-to-Frame (F2F) 2=Fovis 3=viso2 4=DVO-SLAM 5=ORB_SLAM2 6=OKVIS 7=LOAM 8=MSCKF_VIO 9=VINS-Fusion 10=OpenVINS 11=FLOAM 12=Open3D]
        {'Odom/FilteringStrategy': 2},               # [0=No filtering 1=Kalman filtering 2=Particle filtering.]
        # {'Optimizer/Strategy': "1"},                   # Graph optimization strategy: 0=TORO, 1=g2o, 2=GTSAM and 3=Ceres. Only 1 or 2 can be used.
        # {'Optimizer/Robust': True},                  # Robust graph optimization using Vertigo (only work for g2o and GTSAM optimization strategies). Not compatible with {'RGBD/OptimizeMaxError' if enabled.
        # {'Optimizer/GravitySigma': 0.3},
        # {'RGBD/SavedLocalizationIgnored': False},    # Ignore saved localization if the loop closure is detected. This will force RTAB-Map to localize again in the current map.
        # {'Mem/IncrementalMemory': False},            # true = mapping mode
        # {'Mem/ReduceGraph': True},                   # Shuld make the database smaller.
        # {'Mem/SaveDepth16Format': True},             # Save depth images in 16 bit format to save size.
        # {'Mem/UseOdomGravity': True},                # Use odometry instead of IMU orientation to add gravity links to new nodes created.
        # {"publish_tf": True},
    ]

    remappings = [
        ('imu', arg_imu_topic),
        ('rgb/image', arg_rgb_img_topic),
        ('rgb/camera_info', arg_rgb_cam_info_topic),
        ('depth/image', arg_depth_img_topic),
        ('scan', arg_lidar_scan_topic),
        # ('odom', '/odometry/filtered'),
        ('odom', '/okdo/odom'),
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
