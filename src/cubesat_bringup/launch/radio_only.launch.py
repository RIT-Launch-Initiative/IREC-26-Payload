import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_pkg = get_package_share_directory("cubesat_bringup")

    radio_cfg = os.path.join(bringup_pkg, "config", "radio.yaml")

    flight_dir_arg = DeclareLaunchArgument(
        "flight_dir",
        description="Absolute path to the active flight data directory",
    )
    flight_dir = LaunchConfiguration("flight_dir")

    shared = {"flight_dir": flight_dir}


    radio = Node(
        package="cubesat_radio",
        executable="radio_node",
        name="radio_node",
        parameters=[radio_cfg, shared],
        respawn=True,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", "DEBUG"],
    )
    
    return LaunchDescription(
        [
            flight_dir_arg,
            # pi_io,
            # flight_manager,
            # stm_bridge,
            # control,
            radio,
            # vision,
        ]
    )
