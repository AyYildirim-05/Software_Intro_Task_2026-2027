import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('intro_rover_description')
    ps4_config = os.path.join(pkg_share, 'config', 'ps4_controller.yaml')

    # 1. ROS 2 joy_node: reads hardware PS4 controller and publishes /joy (sensor_msgs/msg/Joy)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'deadzone': 0.10, 'autorepeat_rate': 20.0}],
    )

    # 2. joy_demux_node: reads /joy + ps4_controller.yaml, publishes /cmd_vel and controller commands
    joy_demux_node = Node(
        package='intro_rover_description',
        executable='joy_demux_node.py',
        name='joy_demux_node',
        output='screen',
        parameters=[ps4_config],
    )

    return LaunchDescription([
        joy_node,
        joy_demux_node,
    ])
