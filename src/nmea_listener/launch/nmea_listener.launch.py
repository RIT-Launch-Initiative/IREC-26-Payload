from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")
    gps_status_topic = LaunchConfiguration("gps_status_topic")
    max_line_len = LaunchConfiguration("max_line_len")

    return LaunchDescription([
        DeclareLaunchArgument(
            "port",
            default_value="/dev/ttyS0",
            description="Serial device path for the NMEA GPS receiver.",
        ),
        DeclareLaunchArgument(
            "baud",
            default_value="9600",
            description="Baud rate for the GPS serial connection.",
        ),
        DeclareLaunchArgument(
            "gps_status_topic",
            default_value="/gps/status",
            description="Topic on which parsed GPS status messages are published.",
        ),
        DeclareLaunchArgument(
            "max_line_len",
            default_value="512",
            description="Maximum accepted NMEA line length before the data is discarded.",
        ),
        Node(
            package="nmea_listener",
            executable="nmea_listener_node",
            name="nmea_listener",
            output="screen",
            parameters=[
                {
                    "port": port,
                    "baud": ParameterValue(baud, value_type=int),
                    "gps_status_topic": gps_status_topic,
                    "max_line_len": ParameterValue(max_line_len, value_type=int),
                }
            ],
        ),
    ])
