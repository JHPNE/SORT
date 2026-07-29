import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """ROS 2 Launch File for starting TagDetector, WorldSpace, and KinovaMover together."""
    return LaunchDescription([
        Node(
            package='vision_module',
            executable='tag_detector',
            name='tag_detector',
            output='screen'
        ),
        Node(
            package='vision_module',
            executable='world_space',
            name='world_space',
            output='screen'
        ),
        Node(
            package='kinova_controller',
            executable='arm_mover',
            name='kinova_mover',
            output='screen'
        )
    ])
