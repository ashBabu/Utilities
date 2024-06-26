import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def launch_setup(context):
    arg_robot = LaunchConfiguration('robot')  # # iris_with_ouster, iris_with_d455
    robot = arg_robot.perform(context)
    pkg_sensor_sim = get_package_share_directory('sensor_simulation')
    model = os.path.join(pkg_sensor_sim, 'models', robot, 'model.sdf')

    with open(model, 'r') as infp:
        robot_desc = infp.read()

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description":  robot_desc},
            {"frame_prefix": ""},
        ],
    )
    return [node_robot_state_publisher]


def generate_launch_description():

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('robot', default_value='iris_with_ouster'))
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld