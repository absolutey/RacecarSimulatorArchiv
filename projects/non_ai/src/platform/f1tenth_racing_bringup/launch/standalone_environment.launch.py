from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    sim_share = Path(get_package_share_directory("f1tenth_racing_sim"))
    description_share = Path(get_package_share_directory("f1tenth_racing_description"))
    config_path = sim_share / "config/competition_environment.yaml"
    config = yaml.safe_load(config_path.read_text())
    bringup_share = Path(get_package_share_directory("f1tenth_racing_bringup"))
    safety = yaml.safe_load((bringup_share / "config/foundation.yaml").read_text())["runtime_safety"]
    vehicle = config["vehicle"]["description"]
    mappings = {
        "suffix": "0",
        "chassis_length": str(vehicle["chassis_length_m"]),
        "chassis_width": str(vehicle["chassis_width_m"]),
        "chassis_height": str(vehicle["chassis_height_m"]),
        "chassis_z": str(vehicle["chassis_ground_offset_m"]),
        "wheel_radius": str(vehicle["wheel_radius_m"]),
        "wheel_length": str(vehicle["wheel_length_m"]),
        "wheel_x": str(vehicle["wheel_center_x_m"]),
        "wheel_y": str(vehicle["wheel_center_y_m"]),
        "lidar_x": str(vehicle["lidar_x_m"]),
        "lidar_y": str(vehicle["lidar_y_m"]),
        "lidar_z": str(vehicle["lidar_z_m"]),
        "lidar_radius": str(vehicle["lidar_visual_radius_m"]),
        "lidar_height": str(vehicle["lidar_visual_height_m"]),
        "color": " ".join(map(str, vehicle["car0_color_rgba"])),
    }
    robot_description = xacro.process_file(str(description_share / "urdf/vehicle.urdf.xacro"), mappings=mappings).toxml()
    return LaunchDescription([
        Node(package="f1tenth_racing_sim", executable="simulator", output="screen", parameters=[{"command_timeout_s": safety["command_timeout_s"]}]),
        Node(package="robot_state_publisher", executable="robot_state_publisher", name="robot_state_publisher0", parameters=[{"robot_description": robot_description}]),
    ])
