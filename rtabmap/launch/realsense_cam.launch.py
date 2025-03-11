import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    ld = LaunchDescription()

    include_rs_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch/rs_launch.py')),
        launch_arguments={
            "depth_module.profile": "640x480x30",
            "enable_gyro": "true",
            "enable_accel": "true",
            "enable_sync": "true",
            "enable_infra": "true",
            "unite_imu_method": "1",
            "align_depth.enable": "true",
            "camera_namespace": ""
            # "pointcloud.enable": "True",
        }.items()
    )
    ld.add_action(include_rs_cam)

    return ld
