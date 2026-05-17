import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("cubesat_bringup")

    pi_zero_cfg = os.path.join(pkg, "config", "pi_zero.yaml")
    stm_bridge_cfg = os.path.join(pkg, "config", "stm_bridge.yaml")
    radio_cfg = os.path.join(pkg, "config", "radio.yaml")
    vision_cfg = os.path.join(pkg, "config", "vision.yaml")

    flight_dir_arg = DeclareLaunchArgument(
        "flight_dir",
        description="Absolute path to the active flight data directory",
    )
    flight_dir = LaunchConfiguration("flight_dir")

    shared = {"flight_dir": flight_dir}

    # flight_manager = Node(
    #     package="cubesat_flight",
    #     executable="flight_manager_node",
    #     name="flight_manager_node",
    #     parameters=[pi_zero_cfg, shared],
    #     respawn=True,
    #     respawn_delay=2.0,
    #     arguments=["--ros-args", "--log-level", "WARN"],
    # )

    # pi_io = Node(
    #     package="cubesat_pi_io",
    #     executable="pi_io_node",
    #     name="pi_io_node",
    #     parameters=[pi_zero_cfg, shared],
    #     respawn=True,
    #     respawn_delay=2.0,
    #     arguments=["--ros-args", "--log-level", "WARN"],
    # )

    # stm_bridge = Node(
    #     package="cubesat_stm_bridge",
    #     executable="stm_bridge_node",
    #     name="stm_bridge_node",
    #     parameters=[stm_bridge_cfg, shared],
    #     respawn=True,
    #     respawn_delay=2.0,
    #     arguments=["--ros-args", "--log-level", "WARN"],
    # )

    # control = Node(
    #     package="cubesat_control",
    #     executable="control_node",
    #     name="control_node",
    #     parameters=[pi_zero_cfg, shared],
    #     respawn=True,
    #     respawn_delay=2.0,
    #     arguments=["--ros-args", "--log-level", "WARN"],
    # )

    # radio = Node(
    #     package="cubesat_radio",
    #     executable="radio_node",
    #     name="radio_node",
    #     parameters=[radio_cfg, shared],
    #     respawn=True,
    #     respawn_delay=2.0,
    #     arguments=["--ros-args", "--log-level", "WARN"],
    # )

    # vision = Node(
    #     package="cubesat_vision",
    #     executable="vision_node",
    #     name="vision_node",
    #     parameters=[vision_cfg, shared],
    #     respawn=True,
    #     respawn_delay=2.0,
    #     arguments=["--ros-args", "--log-level", "WARN"],
    # )

    return LaunchDescription(
        [
            flight_dir_arg,
            # flight_manager,
            # pi_io,
            # stm_bridge,
            # control,
            # radio,
            # vision,
        ]
    )
