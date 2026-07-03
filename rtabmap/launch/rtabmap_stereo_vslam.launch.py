from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    camera_ns = 'front_stereo_camera'

    arg_base_frame = LaunchConfiguration("base_frame", default="camera_link")
    arg_need_tf = LaunchConfiguration('need_tf', default=True)
    arg_use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    arg_imu_topic = LaunchConfiguration('imu_topic', default='/imu_with_orientation')
    arg_left_img_topic = LaunchConfiguration('left_img_topic',
                                        default='/camera/infra1/image_rect_raw')
    arg_left_cam_info_topic = LaunchConfiguration('left_cam_info_topic', default=f'/{camera_ns}/left/camera_info')
                                        # default='/camera/infra1/camera_info')
    arg_right_img_topic = LaunchConfiguration('right_img_topic',
                                    default='/camera/infra2/image_rect_raw')
    arg_right_cam_info_topic = LaunchConfiguration('right_cam_info_topic', default=f'/{camera_ns}/right/camera_info')
                                        # default='/camera/infra2/camera_info')
    arg_rtab_viz = LaunchConfiguration('rtab_viz', default=False)
    arg_wait_for_imu = LaunchConfiguration('wait_for_imu', default=True)

    arg_color_img_topic = LaunchConfiguration('color_img_topic',
                            default='/camera/color/image_raw')
    arg_color_cam_info_topic = LaunchConfiguration('color_cam_info_topic',
                                default='/camera/color/camera_info')
    arg_depth_img_topic = LaunchConfiguration('depth_img_topic',
                                default='/camera/aligned_depth_to_color/image_raw')

    parameters=[{
          'frame_id': arg_base_frame,
          'subscribe_stereo': True,
          'subscribe_depth': False,
          'subscribe_rgbd': False,
          'approx_sync':True,
          'publish_null_when_lost': True,
          'wait_imu_to_init': arg_wait_for_imu,
          'publish_tf': arg_need_tf,
          'rtabmap_args': '--delete_db_on_start',
          'use_sim_time': arg_use_sim_time}]
    
    remappings=[
          ('imu', arg_imu_topic),
          ('left/image_rect', arg_left_img_topic),
          ('right/image_rect', arg_right_img_topic),
          ('left/camera_info', arg_left_cam_info_topic),
          ('right/camera_info', arg_right_cam_info_topic),
        ]
    
    # if the following params and remappings are used, the rtabmap uses stereo odometry but with colored pointclouds to make the map
    parameters_rgbd_slam = [{
      'frame_id': arg_base_frame,
      'subscribe_depth': True,
      'subscribe_stereo': False,
      'subscribe_rgbd': False,
      'approx_sync': True,
      'wait_imu_to_init': arg_wait_for_imu,
      'publish_tf': arg_need_tf,
      'rtabmap_args': '--delete_db_on_start',
      'use_sim_time': arg_use_sim_time,

        # ---- Hybrid Vis+ICP registration ----
        # 0=Vis only, 1=ICP only, 2=Vis+ICP (visual attempted first, ICP
        # fills in / refines using the depth-derived point cloud). This is
        # the actual switch away from pure feature-based registration.
        'Reg/Strategy': '2',
 
        # Frame-to-Map: match against the accumulated local map rather than
        # only the previous frame. More robust when individual frames are
        # weak on features, which is most frames here.
        'Odom/Strategy': '0',
 
        # ---- ICP tuning (point-to-plane suits the pipe's smooth, curved
        # surface better than point-to-point, since it uses local normals) ----
        'Icp/PointToPlane': 'true',
        'Icp/PointToPlaneK': '20',
        'Icp/PointToPlaneRadius': '0',
        'Icp/VoxelSize': '0.02',          # downsample cloud before matching (m); tune to pipe scale
        'Icp/MaxCorrespondenceDistance': '0.1',  # (m); tune to pipe diameter
        'Icp/Iterations': '30',
        'Icp/Epsilon': '0.001',
        'Icp/MaxTranslation': '0.5',      # allow larger per-step motion than default
 
        # ---- Vis tuning: relax feature requirements rather than let a
        # feature-starved frame reject visual registration outright; ICP
        # picks up the slack via Reg/Strategy=2 either way ----
        'Vis/MinInliers': '10',
        'Vis/InlierDistanceRatio': '0.1',
 
        # Extra proximity-based link refinement helps in corridor-like,
        # repetitive-geometry environments (pipes) to curb drift.
        'RGBD/NeighborLinkRefining': 'true',
    }]
    
    remappings_rgbd_slam = [
      ('imu', arg_imu_topic),
      ('rgb/image', arg_color_img_topic),
      ('depth/image', arg_depth_img_topic),
      ('rgb/camera_info', arg_color_cam_info_topic),
    ]

    node_decompress_left = Node(
        package='image_transport',
        executable='republish',
        name='decompress_left',
        parameters=[{
            'use_sim_time':  arg_use_sim_time,
            'in_transport':  'compressed',
            'out_transport': 'raw',
        }],
        remappings=[
            ('in/compressed', f'/{camera_ns}/left/image_compressed'),
            ('out', f'/{camera_ns}/left/image_raw'),
        ],
        output='screen'
    )
    # ld.add_action(node_decompress_left)

    node_decompress_right = Node(
        package='image_transport',
        executable='republish',
        name='decompress_right',
        parameters=[{
            'use_sim_time':  arg_use_sim_time,
            'in_transport':  'compressed',
            'out_transport': 'raw',
        }],
        remappings=[
            ('in/compressed', f'/{camera_ns}/right/image_compressed'),
            ('out', f'/{camera_ns}/right/image_raw'),
        ],
        output='screen'
    )
    # ld.add_action(node_decompress_right)

    node_rectify_left = Node(
        package='image_proc', executable='rectify_node', name='rectify_left',
        remappings=[
            ('image', f'/{camera_ns}/left/image_raw'),
            ('camera_info', f'/{camera_ns}/left/camera_info'),
            ('image_rect', '/camera/infra1/image_rect_raw'),
        ],
        parameters=[{'use_sim_time': arg_use_sim_time}],
    )
    # ld.add_action(node_rectify_left)
 
    node_rectify_right = Node(
        package='image_proc', executable='rectify_node', name='rectify_right',
        remappings=[
            ('image', f'/{camera_ns}/right/image_raw'),
            ('camera_info', f'/{camera_ns}/right/camera_info'),
            ('image_rect', '/camera/infra2/image_rect_raw'),
        ],
        parameters=[{'use_sim_time': arg_use_sim_time}],
    )
    # ld.add_action(node_rectify_right)
    
    node_rtabmap_odom = Node(
        name='rtabmap_odom',
        package='rtabmap_odom', executable='stereo_odometry', output='screen',
        parameters=parameters,
        remappings=remappings
    )
    ld.add_action(node_rtabmap_odom)

    node_rtabmap_slam = Node(
        name='rtabmap_slam',
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=parameters_rgbd_slam,
        remappings=remappings_rgbd_slam,
        arguments=['-d']
    )
    ld.add_action(node_rtabmap_slam)

    node_rtabmap_viz = Node(
        condition=IfCondition(arg_rtab_viz),
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        parameters=parameters,
        remappings=remappings
    )
    ld.add_action(node_rtabmap_viz)

    return ld
