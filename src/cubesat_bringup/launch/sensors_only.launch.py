import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_pkg = get_package_share_directory("cubesat_bringup")
    pi_io_pkg = get_package_share_directory("cubesat_pi_io")

    pi_io_cfg = os.path.join(pi_io_pkg, "config", "pi_io.yaml")
    stm_bridge_cfg = os.path.join(bringup_pkg, "config", "stm_bridge.yaml")

    flight_dir_arg = DeclareLaunchArgument(
        "flight_dir",
        default_value="/tmp/sensor_debug",
        description="Flight data directory for sensor debug output",
    )
    flight_dir = LaunchConfiguration("flight_dir")

    shared = {"flight_dir": flight_dir}

    pi_io = Node(
        package="cubesat_pi_io",
        executable="pi_io_node",
        name="pi_io_node",
        parameters=[pi_io_cfg, shared],
        respawn=False,
        arguments=["--ros-args", "--log-level", "DEBUG"],
    )

    # Uncomment once cubesat_stm_bridge exists.
    # stm_bridge = Node(
    #     package="cubesat_stm_bridge",
    #     executable="stm_bridge_node",
    #     name="stm_bridge_node",
    #     parameters=[stm_bridge_cfg, shared],
    #     respawn=False,
    #     arguments=["--ros-args", "--log-level", "DEBUG"],
    # )

    return LaunchDescription(
        [
            flight_dir_arg,
            pi_io,
            # stm_bridge,
        ]
    )
