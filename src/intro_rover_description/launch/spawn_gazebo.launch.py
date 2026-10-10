
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, LogInfo
import shutil


def generate_launch_description():
    pkg_share = get_package_share_directory('intro_rover_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'intro_rover_description.urdf')
    gz_resource_root = pkg_share
    existing_gz_resources = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = os.pathsep.join([
        gz_resource_root,
        existing_gz_resources,
    ]) if existing_gz_resources else gz_resource_root

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    set_gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        gz_resource_path,
    )

    # robot_state_publisher provides TFs for visualizers and Gazebo plugins
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Use the ros_gz_sim create binary directly. The CLI supports loading XML
    # from a file and is the correct path for Gazebo Sim on ROS 2 Jazzy.
    spawn = None
    try:
        get_package_share_directory('ros_gz_sim')
        spawn = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-world', 'empty', '-file', urdf_file, '-name', 'intro_rover', '-x', '0.0', '-y', '0.0', '-z', '0.5'],
            output='screen'
        )
    except Exception:
        ros2_exe = shutil.which('ros2')
        if ros2_exe:
            spawn = ExecuteProcess(
                cmd=[ros2_exe, 'run', 'ros_gz_sim', 'create', '-world', 'empty', '-file', urdf_file, '-name', 'intro_rover', '-x', '0.0', '-y', '0.0', '-z', '0.5'],
                output='screen'
            )
        else:
            spawn = None

    # fallback for classic gazebo (spawn_entity.py) if ros_gz isn't usable
    if spawn is None:
        try:
            # check gazebo_ros package exists
            get_package_share_directory('gazebo_ros')
            spawn = Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-topic', 'robot_description', '-entity', 'intro_rover'],
                output='screen'
            )
        except Exception:
            # Last resort: log an informational message so the user sees why
            # no spawn will be attempted.
            spawn = LogInfo(msg=['No spawn method available: neither ros_gz nor gazebo_ros appears usable. Start gz-sim and then run a manual spawn.'])

    # Delay spawn to give the simulator time to start
    delayed_spawn = TimerAction(period=5.0, actions=[spawn])

    return LaunchDescription([
        set_gz_resource_path,
        rsp_node,
        delayed_spawn,
    ])
