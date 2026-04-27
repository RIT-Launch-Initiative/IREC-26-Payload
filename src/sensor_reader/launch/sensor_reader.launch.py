from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    read_interval_ms = LaunchConfiguration("read_interval_ms")
    adxl375_topic = LaunchConfiguration("adxl375_topic")
    adxl375_i2c_dev = LaunchConfiguration("adxl375_i2c_dev")
    adxl375_addr = LaunchConfiguration("adxl375_addr")
    ina260_topic = LaunchConfiguration("ina260_topic")
    ina260_i2c_dev = LaunchConfiguration("ina260_i2c_dev")
    ina260_addr = LaunchConfiguration("ina260_addr")

    return LaunchDescription([
        DeclareLaunchArgument(
            "read_interval_ms",
            default_value="1000",
            description="Interval between successive sensor reads (milliseconds).",
        ),
        DeclareLaunchArgument(
            "adxl375_topic",
            default_value="/sensor_msgs/msg/Imu",
            description="Topic on which ADXL375 IMU data is published.",
        ),
        DeclareLaunchArgument(
            "adxl375_i2c_dev",
            default_value="/dev/i2c-1",
            description="I2C device path for the ADXL375.",
        ),
        DeclareLaunchArgument(
            "adxl375_addr",
            default_value="83",
            description="I2C address for the ADXL375 accelerometer (decimal).",
        ),
        DeclareLaunchArgument(
            "ina260_topic",
            default_value="/sensor_msgs/msg/BatteryState",
            description="Topic on which INA260 readings are published.",
        ),
        DeclareLaunchArgument(
            "ina260_i2c_dev",
            default_value="/dev/i2c-1",
            description="I2C device path for the INA260.",
        ),
        DeclareLaunchArgument(
            "ina260_addr",
            default_value="64",
            description="I2C address for the INA260 sensor (decimal).",
        ),
        Node(
            package="sensor_reader",
            executable="sensor_reader_node",
            name="sensor_reader",
            output="screen",
            parameters=[
                {
                    "read_interval_ms": ParameterValue(read_interval_ms, value_type=int),
                    "adxl375_topic": adxl375_topic,
                    "adxl375_i2c_dev": adxl375_i2c_dev,
                    "adxl375_addr": ParameterValue(adxl375_addr, value_type=int),
                    "ina260_topic": ina260_topic,
                    "ina260_i2c_dev": ina260_i2c_dev,
                    "ina260_addr": ParameterValue(ina260_addr, value_type=int),
                }
            ],
        ),
    ])
