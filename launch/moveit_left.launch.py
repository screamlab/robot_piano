import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushROSNamespace


def generate_launch_description():
    moveit_left = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robot_piano'), 'launch'),
            '/robot_moveit.launch.py'])
        )
    moveit_left_with_namespace = GroupAction(
        actions=[
            PushROSNamespace('left'),
            moveit_left,
        ]
    )

    return LaunchDescription([
        moveit_left_with_namespace,
    ])
