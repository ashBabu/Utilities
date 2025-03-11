import os
from datetime import datetime
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LoadComposableNodes, SetParameter, ComposableNodeContainer


def generate_launch_description():

    ld = LaunchDescription()
    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    database_path = os.path.expanduser(f'~/.rtabmap/slam_{timestamp}.db')

    arg_need_tf = LaunchConfiguration('need_tf', default=True)
    arg_rgbd_odom_pub_topic = LaunchConfiguration('rgbd_odom_pub_topic')
    arg_base_frame = LaunchConfiguration('base_frame', default='base_link')
    arg_imu_topic = LaunchConfiguration(
        'imu_topic', default='/camera/imu/data')
    arg_rgb_img_topic = LaunchConfiguration(
        'rgb_img_topic', default='/camera/color/image_raw')
    arg_rgb_cam_info_topic = LaunchConfiguration(
        'rgb_cam_info_topic', default='/camera/color/camera_info')
    arg_depth_img_topic = LaunchConfiguration(
        'depth_img_topic', default='/camera/aligned_depth_to_color/image_raw')
    arg_rgbd_odom_pub_topic = LaunchConfiguration(
        'rgbd_odom_pub_topic', default='/rtabmap/realsense/rgbd/odom')

    # Tune rtabmap parameters:
        # http://wiki.ros.org/rtabmap_ros/Tutorials/Advanced%20Parameter%20Tuning#Visual_Odometry
    # ICP odometry wont run with realsense D435i :
        # https://github.com/introlab/rtabmap_ros/issues/1061#issuecomment-1806942969
    # Rtabmap Odometry examples:
        # https://github.com/introlab/rtabmap_ros/tree/ros2/rtabmap_examples/launch
    # Parameters
        # https://github.com/introlab/rtabmap/blob/jazzy-devel/corelib/include/rtabmap/core/Parameters.h
    parameters = [
        {'rtabmap_args': '--delete_db_on_start'},
        {'database_path': database_path},
        {'subscribe_depth': "True"},
        {'subscribe_odom_info': "True"},
        {'approx_sync': Fa"lse},
        {"frame_id": arg_base_frame},
        {"odom_frame_id": "odom"},
        {"publish_tf": arg_need_tf},
        {"wait_imu_to_init": True},
        {"queue_size": 50},
        {"qos": 2},
        {"qos_imu": 2},
        # {"publish_tf_map": True},
        {"Odom/Strategy": "1"},
        {"OdomF2M/MaxSize": "1000"},
        {"Odom/ResetCountdown": "2"},
        {"Odom/GuessSmoothingDelay": "0"},
        {"Rtabmap/StartNewMapOnLoopClosure": "False"},
        {"Rtabmap/CreateIntermediateNodes": "True"},
        {"RGBD/CreateOccupancyGrid": "False"},
        # {"RGBD/LinearUpdate": "False"},
        # {"RGBD/AngularUpdate": "False"},
        {"Grid/RangeMax": "20.0"},
        {"Vis/EstimationType": "1"}, #    "Motion estimation approach: 0:3D->3D, 1:3D->2D (PnP), 2:2D->2D (Epipolar Geometry)");
        {"Vis/CorType": "1"},
        {"Reg/Force3DoF": "True"},
        {"Mem/PublishTrajectory": "True"},
        {"Kp/DetectorStrategy": "2"}  #  "0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint 12=SURF/FREAK 13=GFTT/DAISY 14=SURF/DAISY 15=PyDetector");
    ]

    remappings = [
        ('imu', arg_imu_topic),
        ('rgb/image', arg_rgb_img_topic),
        ('rgb/camera_info', arg_rgb_cam_info_topic),
        ('depth/image', arg_depth_img_topic),
        ('odom', arg_rgbd_odom_pub_topic)
    ]

    set_param = SetParameter(name='depth_module.emitter_enabled', value=1)
    ld.add_action(set_param)

    node_rtabmap_rgbd_odom = Node(
            package="rtabmap_odom",
            executable="rgbd_odometry",
            name="rgbd_odometry",
            namespace='/rtabamp',
            output="screen",
            parameters=parameters,
            remappings=remappings
        )
    ld.add_action(node_rtabmap_rgbd_odom)

    return ld
