import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # Packages we are using
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')
    pkg_dd_robot = get_package_share_directory('dd_robot')

    # Define the robots that we want to spawn in
    robots = [
        { 'name': 'pursuer_2', 'x': '5.0', 'y': '5.0' },
        { 'name': 'evader',  'x': '5.0', 'y': '0.0' }
    ]

    # Initialize custom robot based on urdf file
    urdf_file = os.path.join(pkg_dd_robot, 'urdf', 'dd_robot.urdf.xacro')

    # Define the gazebo launch command
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_ros_gz, 'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments = {
            'gz_args': ['-r ', os.path.join(pkg_dd_robot, 'worlds', 'pursuit_world.sdf')],
            'on_exit_shutdown': 'True'
        }.items()
    )

    manager = Node(
        package = 'pursuit_evasion',
        executable = 'manager',
        name = 'manager',
        output = 'screen',
        parameters = [{
            'active_robots': [r['name'] for r in robots]
        }]
    )

    launch_items = [
        gazebo,
        manager
    ]
    
    for robot in robots:

        robot_name = robot['name']

        robot_description = Command([
            'xacro ',
            urdf_file,
            ' robot_name:=', robot_name
        ])

        robot_state_publisher = Node(
            package = 'robot_state_publisher',
            executable = 'robot_state_publisher',
            namespace = robot_name,
            name = 'robot_state_publisher',
            output = 'screen',
            parameters = [{
                'robot_description': robot_description,
                'frame_prefix': f'{robot_name}/'
            }]
        )

        bridge = Node(
            package = 'ros_gz_bridge',
            executable = 'parameter_bridge',
            namespace = robot_name,
            arguments = [
                f'/model/{robot_name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                f'/model/{robot_name}/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
            ],
            remappings = [
                (f'/model/{robot_name}/cmd_vel', f'/{robot_name}/cmd_vel'),
                (f'/model/{robot_name}/pose', f'/{robot_name}/pose'),
            ]
        )

        spawn = Node(
            package = 'ros_gz_sim',
            executable = 'create',
            name = 'spawn_dd_robot',
            arguments = [
                '-name', robot_name,
                '-topic', f'/{robot_name}/robot_description',
                '-x', robot['x'],
                '-y', robot['y'],
                '-z', '0.5',
                '-Y', '0'
            ],
            output = 'screen'
        )

        if robot_name.split('_')[0] == 'pursuer':
            controller = Node(
                package = 'pursuit_evasion',
                executable = 'pursuit_controller',
                namespace = robot_name,
                name = f'{robot_name}_controller',
                output = 'screen',
                parameters = [{
                    'robot': f'{robot_name}'
                }]
            )
            launch_items.append(controller)

        elif robot_name == 'evader':
            controller = Node(
                package = 'pursuit_evasion',
                executable = 'evasion_controller',
                namespace = robot_name,
                name = f'{robot_name}_controller',
                output = 'screen',
                parameters = [{
                    'robot': f'{robot_name}'
                }]
            )
            launch_items.append(controller)

        launch_items.append(robot_state_publisher)
        launch_items.append(bridge)
        launch_items.append(spawn) 

    return LaunchDescription(
        launch_items
    )
