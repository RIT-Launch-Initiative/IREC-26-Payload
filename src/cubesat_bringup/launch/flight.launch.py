import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

'''
TODO TODO TODO
- start ros bag

'''


def generate_launch_description():
    bringup_pkg = get_package_share_directory("cubesat_bringup")
    pi_io_pkg = get_package_share_directory("cubesat_pi_io")

    captain_cfg = os.path.join(bringup_pkg, "config", "captain.yaml")

    pi_io_cfg = os.path.join(pi_io_pkg, "config", "pi_io.yaml")
    stm_bridge_cfg = os.path.join(bringup_pkg, "config", "stm_bridge.yaml")
    radio_cfg = os.path.join(bringup_pkg, "config", "radio.yaml")

    flight_dir_arg = DeclareLaunchArgument(
        "flight_dir",
        description="Absolute path to the active flight data directory",
    )
    flight_dir = LaunchConfiguration("flight_dir")

    shared = {"flight_dir": flight_dir}

    pi_io = Node(
        package="cubesat_pi_io",
        executable="pi_io_node",
        name="pi_io_node",
        parameters=[pi_io_cfg, shared],
        respawn=True,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    captain = Node(
        package="cubesat_captain",
        executable="captain_node",
        name="captain_node",
        parameters=[captain_cfg, shared],
        respawn=True,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    stm_bridge = Node(
        package="cubesat_stm_bridge",
        executable="stm_bridge_node",
        name="stm_bridge_node",
        parameters=[stm_bridge_cfg, shared],
        respawn=True,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    # control = Node(
    #     package="cubesat_control",
    #     executable="control_node",
    #     name="control_node",
    #     parameters=[shared],
    #     respawn=True,
    #     respawn_delay=2.0,
    #     arguments=["--ros-args", "--log-level", "WARN"],
    # )

    radio = Node(
        package="cubesat_radio",
        executable="radio_node",
        name="radio_node",
parameters=[radio_cfg, shared],
        respawn=True,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    # 1280, 960 def works
    camera = Node(
            package="camera_ros",
            executable="camera_node",
            name="camera",
            output="screen",
            respawn=True,
            respawn_delay=20.0,
            parameters=[{
                "camera": "/base/soc/i2c0mux/i2c@1/imx219@10",
                "width": 1280,
                "height": 800,
                "format": "YUYV",
                "FrameDurationLimits": [500000,500000],
                "AeEnabled": True,
                "AwbEnabled": True,
                "AfMode": "Continuous",
            }],
            arguments=["--ros-args", "--log-level", "WARN"],
        )
    
    watch = Node(
            package="cubesat_watch",
            executable="cubesat_watch_node",
            name="watch",
            output="screen",
            parameters = [shared],
            arguments=["--ros-args", "--log-level", "INFO"],

        )
    

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
            pi_io,
            captain,
            stm_bridge,
            # control,
            radio,
            camera,
            watch
        ]
    )
