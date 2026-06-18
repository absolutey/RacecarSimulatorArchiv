from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("race_driver")
    config_file = os.path.join(pkg_dir, "config", "race_driver.yaml")

    race_driver_node = Node(
        package="race_driver",
        executable="race_driver_node",
        name="race_driver_node",
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription([
        race_driver_node,
    ])
