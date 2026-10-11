import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('intro_rover_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    urdf_source_file = os.path.join(pkg_share, 'urdf', 'intro_rover_description.urdf')
    controllers_yaml_path = os.path.join(pkg_share, 'config', 'rover_controllers.yaml')

    with open(urdf_source_file, 'r') as f:
        urdf_content = f.read()

    urdf_content = urdf_content.replace(
        '$(find intro_rover_description)/config/rover_controllers.yaml',
        controllers_yaml_path
    )

    temp_urdf = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.urdf')
    temp_urdf.write(urdf_content)
    temp_urdf.flush()
    processed_urdf_file = temp_urdf.name

    workspace_share = os.path.dirname(pkg_share)
    existing_gz_resources = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = os.pathsep.join([
        workspace_share,
        existing_gz_resources,
    ]) if existing_gz_resources else workspace_share

    set_gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        gz_resource_path,
    )

    # 1. Start Gazebo Sim with empty world unpaused (-r)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # 2. Bridge Gazebo Clock to ROS Clock
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # 3. Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf_content, 'use_sim_time': True}]
    )

    # 4. Spawn rover entity into Gazebo Sim
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'empty',
            '-file', processed_urdf_file,
            '-name', 'intro_rover',
            '-x', '0.0', '-y', '0.0', '-z', '0.5'
        ],
        output='screen'
    )

    delayed_spawn = TimerAction(period=3.0, actions=[spawn])

    # 5. Controller Spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    wheel_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheel_velocity_controller'],
        output='screen'
    )

    steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['steering_position_controller'],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen'
    )

    load_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )

    load_controllers_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                wheel_controller_spawner,
                steering_controller_spawner,
                arm_controller_spawner,
            ]
        )
    )

    return LaunchDescription([
        set_gz_resource_path,
        gz_sim,
        clock_bridge,
        rsp_node,
        delayed_spawn,
        load_jsb_after_spawn,
        load_controllers_after_jsb,
    ])
