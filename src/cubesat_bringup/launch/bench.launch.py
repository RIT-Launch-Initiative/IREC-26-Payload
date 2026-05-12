import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory("cubesat_bringup")
    pi_io_pkg = get_package_share_directory("cubesat_pi_io")

    pi_io_cfg = os.path.join(pi_io_pkg, "config", "pi_io.yaml")
    stm_bridge_cfg = os.path.join(bringup_pkg, "config", "stm_bridge.yaml")

    flight_dir_arg = DeclareLaunchArgument(
        "flight_dir",
        default_value="/tmp/bench_flight",
        description="Flight data directory (defaults to /tmp for bench runs)",
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

    # flight_manager = Node(
    #     package="cubesat_flight",
    #     executable="flight_manager_node",
    #     name="flight_manager_node",
    #     parameters=[shared],
    #     respawn=False,
    #     arguments=["--ros-args", "--log-level", "DEBUG"],
    # )

    # stm_bridge = Node(
    #     package="cubesat_stm_bridge",
    #     executable="stm_bridge_node",
    #     name="stm_bridge_node",
    #     parameters=[stm_bridge_cfg, shared],
    #     respawn=False,
    #     arguments=["--ros-args", "--log-level", "DEBUG"],
    # )

    # control = Node(
    #     package="cubesat_control",
    #     executable="control_node",
    #     name="control_node",
    #     parameters=[shared],
    #     respawn=False,
    #     arguments=["--ros-args", "--log-level", "DEBUG"],
    # )

    # Radio and vision are excluded from bench.

    return LaunchDescription(
        [
            flight_dir_arg,
            pi_io,
            # flight_manager,
            # stm_bridge,
            # control,
        ]
    )
