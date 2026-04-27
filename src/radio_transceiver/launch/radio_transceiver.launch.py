from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    spi_device = LaunchConfiguration("spi_device")
    gpio_chip_name = LaunchConfiguration("gpio_chip_name")
    reset_gpio_line = LaunchConfiguration("reset_gpio_line")
    busy_gpio_line = LaunchConfiguration("busy_gpio_line")
    spi_speed_hz = LaunchConfiguration("spi_speed_hz")
    tx_interval_ms = LaunchConfiguration("tx_interval_ms")
    rx_poll_interval_ms = LaunchConfiguration("rx_poll_interval_ms")
    tx_timeout_ms = LaunchConfiguration("tx_timeout_ms")
    gs_timeout_sec = LaunchConfiguration("gs_timeout_sec")
    radio_revert_sec = LaunchConfiguration("radio_revert_sec")
    tx_port = LaunchConfiguration("tx_port")
    rx_move_arm_port = LaunchConfiguration("rx_move_arm_port")
    rx_set_radio_params_port = LaunchConfiguration("rx_set_radio_params_port")
    frequency_hz = LaunchConfiguration("frequency_hz")
    tx_power_dbm = LaunchConfiguration("tx_power_dbm")
    spreading_factor = LaunchConfiguration("spreading_factor")
    bandwidth_hz = LaunchConfiguration("bandwidth_hz")
    coding_rate = LaunchConfiguration("coding_rate")
    preamble_length = LaunchConfiguration("preamble_length")
    heartbeat_topic = LaunchConfiguration("heartbeat_topic")
    arm_status_topic = LaunchConfiguration("arm_status_topic")
    arm_command_topic = LaunchConfiguration("arm_command_topic")
    arm_target_topic = LaunchConfiguration("arm_target_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "spi_device",
            default_value="/dev/spidev0.0",
            description="SPI device used to communicate with the SX126x radio.",
        ),
        DeclareLaunchArgument(
            "gpio_chip_name",
            default_value="gpiochip0",
            description="GPIO chip name containing reset and busy lines.",
        ),
        DeclareLaunchArgument(
            "reset_gpio_line",
            default_value="31",
            description="GPIO line number for the radio reset pin.",
        ),
        DeclareLaunchArgument(
            "busy_gpio_line",
            default_value="12",
            description="GPIO line number for the radio busy pin.",
        ),
        DeclareLaunchArgument(
            "spi_speed_hz",
            default_value="1000000",
            description="SPI clock speed for radio communication (Hz).",
        ),
        DeclareLaunchArgument(
            "tx_interval_ms",
            default_value="5000",
            description="Interval between outbound transmissions (milliseconds).",
        ),
        DeclareLaunchArgument(
            "rx_poll_interval_ms",
            default_value="50",
            description="Polling interval for inbound radio traffic (milliseconds).",
        ),
        DeclareLaunchArgument(
            "tx_timeout_ms",
            default_value="3000",
            description="Timeout for completing a transmit operation (milliseconds).",
        ),
        DeclareLaunchArgument(
            "gs_timeout_sec",
            default_value="60",
            description="Timeout before considering ground-station interaction stale (seconds).",
        ),
        DeclareLaunchArgument(
            "radio_revert_sec",
            default_value="30",
            description="Delay before reverting temporary radio config (seconds).",
        ),
        DeclareLaunchArgument(
            "tx_port",
            default_value="9000",
            description="Application port used for outbound radio frames.",
        ),
        DeclareLaunchArgument(
            "rx_move_arm_port",
            default_value="9100",
            description="Port for inbound move-arm commands.",
        ),
        DeclareLaunchArgument(
            "rx_set_radio_params_port",
            default_value="9101",
            description="Port for inbound radio configuration commands.",
        ),
        DeclareLaunchArgument(
            "frequency_hz",
            default_value="915000000",
            description="Operating frequency of the radio (Hz).",
        ),
        DeclareLaunchArgument(
            "tx_power_dbm",
            default_value="22",
            description="Transmit power setting (dBm).",
        ),
        DeclareLaunchArgument(
            "spreading_factor",
            default_value="12",
            description="LoRa spreading factor.",
        ),
        DeclareLaunchArgument(
            "bandwidth_hz",
            default_value="125000",
            description="LoRa bandwidth (Hz).",
        ),
        DeclareLaunchArgument(
            "coding_rate",
            default_value="5",
            description="LoRa coding rate denominator (e.g., 5 represents 4/5).",
        ),
        DeclareLaunchArgument(
            "preamble_length",
            default_value="8",
            description="LoRa preamble length.",
        ),
        DeclareLaunchArgument(
            "heartbeat_topic",
            default_value="heartbeat_status",
            description="Topic carrying heartbeat telemetry to transmit.",
        ),
        DeclareLaunchArgument(
            "arm_status_topic",
            default_value="arm_status",
            description="Topic carrying arm status telemetry to transmit.",
        ),
        DeclareLaunchArgument(
            "arm_command_topic",
            default_value="arm_command",
            description="Topic carrying locally issued arm commands.",
        ),
        DeclareLaunchArgument(
            "arm_target_topic",
            default_value="/arm_target",
            description="Topic to publish decoded move-arm targets for downstream nodes.",
        ),
        Node(
            package="radio_transceiver",
            executable="radio_transceiver_node",
            name="radio_transceiver",
            output="screen",
            parameters=[
                {
                    "spi_device": spi_device,
                    "gpio_chip_name": gpio_chip_name,
                    "reset_gpio_line": ParameterValue(reset_gpio_line, value_type=int),
                    "busy_gpio_line": ParameterValue(busy_gpio_line, value_type=int),
                    "spi_speed_hz": ParameterValue(spi_speed_hz, value_type=int),
                    "tx_interval_ms": ParameterValue(tx_interval_ms, value_type=int),
                    "rx_poll_interval_ms": ParameterValue(rx_poll_interval_ms, value_type=int),
                    "tx_timeout_ms": ParameterValue(tx_timeout_ms, value_type=int),
                    "gs_timeout_sec": ParameterValue(gs_timeout_sec, value_type=int),
                    "radio_revert_sec": ParameterValue(radio_revert_sec, value_type=int),
                    "tx_port": ParameterValue(tx_port, value_type=int),
                    "rx_move_arm_port": ParameterValue(rx_move_arm_port, value_type=int),
                    "rx_set_radio_params_port": ParameterValue(
                        rx_set_radio_params_port, value_type=int
                    ),
                    "frequency_hz": ParameterValue(frequency_hz, value_type=int),
                    "tx_power_dbm": ParameterValue(tx_power_dbm, value_type=int),
                    "spreading_factor": ParameterValue(spreading_factor, value_type=int),
                    "bandwidth_hz": ParameterValue(bandwidth_hz, value_type=int),
                    "coding_rate": ParameterValue(coding_rate, value_type=int),
                    "preamble_length": ParameterValue(preamble_length, value_type=int),
                    "heartbeat_topic": heartbeat_topic,
                    "arm_status_topic": arm_status_topic,
                    "arm_command_topic": arm_command_topic,
                    "arm_target_topic": arm_target_topic,
                }
            ],
        ),
    ])
