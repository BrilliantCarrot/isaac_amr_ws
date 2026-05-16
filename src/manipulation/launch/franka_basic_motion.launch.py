#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("manipulation")
    config_path = os.path.join(pkg_share, "config", "franka_basic_motion.yaml")

    declare_auto_start = DeclareLaunchArgument(
        "auto_start",
        default_value="true",
        description="true이면 launch 직후 basic pick/place joint 시퀀스를 1회 실행",
    )

    auto_start = LaunchConfiguration("auto_start")

    franka_basic_motion_node = Node(
        package="manipulation",
        executable="franka_basic_motion_node",
        name="franka_basic_motion_node",
        output="screen",
        parameters=[
            config_path,
            {"auto_start": auto_start},
        ],
    )

    return LaunchDescription([
        declare_auto_start,
        franka_basic_motion_node,
    ])
