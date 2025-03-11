# This is the launch file for mtc.cpp node
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    ik_calc = Node(
        package="robot_piano",
        executable="ik_calc",
    )

    return LaunchDescription([ik_calc])
