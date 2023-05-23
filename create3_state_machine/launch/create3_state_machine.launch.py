import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    create3_state_machine_path = get_package_share_directory(
        'create3_state_machine')

    # Rviz
    rviz_config = PathJoinSubstitution(
        [create3_state_machine_path, 'rviz', 'create3_state_machine.rviz'])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '--display-config', rviz_config
        ]
    )
    create_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([get_package_share_directory('irobot_create_gazebo_bringup'),
                                  'launch', 'create3_gazebo.launch.py'])),
        launch_arguments={'use_rviz': 'false',
                          'use_gazebo_gui': 'false'}.items()

    )
    create3_state_machine_node = Node(
        package='create3_state_machine',
        executable='create3_state_machine',
        name='create3_state_machine',
        output='screen'
    )

    return LaunchDescription([
        rviz,
        create_gazebo,
        create3_state_machine_node
    ])
