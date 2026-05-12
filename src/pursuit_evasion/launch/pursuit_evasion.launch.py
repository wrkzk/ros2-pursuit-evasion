import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_ros_gz = get_package_share_directory('ros_gz_sim')
    pkg_dd_robot = get_package_share_directory('dd_robot')

    # Initialize custom robot based on urdf file
    urdf_file = os.path.join(pkg_dd_robot, 'urdf', 'dd_robot.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_ros_gz, 'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments = {
            'gz_args': [' -r'],
            'on_exit_shutdown': 'True'
        }.items()
    )

    robot_state_publisher = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        namespace = 'dd_robot',
        name = 'robot_state_publisher',
        output = 'screen',
        parameters = [{
            'robot_description': robot_description,
            'use_sim_time': True,
            'frame_prefix': 'dd_robot/'
        }],
        remappings = [
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    spawn = Node(
        package = 'ros_gz_sim',
        executable = 'create',
        name = 'spawn_dd_robot',
        arguments = [
            '-name', 'dd_robot',
            '-topic', '/dd_robot/robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0',
            '-Y', '0'
        ],
        output = 'screen'
    )

    return LaunchDescription(
        [
            gazebo,
            robot_state_publisher,
            spawn
        ]
    )
