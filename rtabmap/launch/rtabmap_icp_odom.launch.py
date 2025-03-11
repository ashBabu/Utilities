from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    ld = LaunchDescription()

    arg_scan_topic = LaunchConfiguration('lidar_scan_topic')
    arg_odom_topic = LaunchConfiguration('lidar_odom_topic')
    arg_base_frame = LaunchConfiguration('base_frame')
    arg_need_tf = LaunchConfiguration('need_tf', default=True)
    arg_sim = LaunchConfiguration('sim', default=False)

    parameters = [{
        "frame_id": arg_base_frame,  # Robot base frame
        # "frame_id": "base_laser",  # Robot base frame
        "odom_frame_id": "odom",  # Odometry frame
        "publish_tf": arg_need_tf,  # robot_localization would publish this
        "approx_sync": True,
        # "wait_for_transform": True,
        'subscribe_scan': True,
        "use_sim_time": arg_sim,

        # ICP parameters
        # Enable Point-to-Plane ICP (better for structured environments)
        "icp/pointToPlane": True,
        "icp/iterations": 30,      # Number of ICP iterations per scan
        "icp/maxCorrespondenceDistance": 0.2,  # Max distance for point matching
        "icp/downsamplingStep": 2,  # Reduce point cloud density for efficiency
        # Ratio of matched points to accept a transformation
        "icp/correspondenceRatio": 0.3,
        "icp/voxelSize": 0.05,  # Set voxel size for ICP (meters)

        # Odometry filtering & robustness
        "reg/strategy": 1,  # Use ICP instead of visual features
        # Keyframe threshold (higher = fewer keyframes)
        "odom/scanKeyFrameThr": 0.6,
        "odom/guessMotion": True,  # Use constant velocity model for prediction
        "Odom/Strategy": "0",
        # "Vis/CorType": "1",  # 0=Features Matching, 1=Optical Flow
        "OdomF2M/MaxSize": "1000",  # maximum features map size, default 2000
        "Odom/ResetCountdown": "1",
        # https://answers.ros.org/question/250528/rtabmap-how-to-save-registered-3d-point-cloud/
        "Grid/FromDepth": "false",
        "Reg/Force3DoF": "true",          # for 2D
        # "Optimizer/Slam2D": "true",        # for 2D
    }]

    remappings = [
        ("/scan", arg_scan_topic),  
        ('odom', arg_odom_topic)  
    ]

    node_rtabmap_icp_odom = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=parameters,
        remappings=remappings
    )
    ld.add_action(node_rtabmap_icp_odom)

    return ld