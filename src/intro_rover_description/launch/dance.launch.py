import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('intro_rover_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'intro_rover_description.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # 1. Publishes 3D transforms (/tf) from URDF + joint positions
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # 2. Your dance choreography node streaming to /joint_states
        Node(
            package='intro_rover_description',
            executable='jointstate.py',
            name='rover_dancer',
            output='screen'
        ),

        # 3. RViz visualizer
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])