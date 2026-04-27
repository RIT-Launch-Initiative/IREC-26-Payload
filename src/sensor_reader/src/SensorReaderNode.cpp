#include "sensor_reader/SensorReaderNode.hpp"

SensorReaderNode::SensorReaderNode()
    : Node("sensor_reader_node"),
      readIntervalMs(this->declare_parameter<int>("read_interval_ms", 100)),
      adxl375Topic(this->declare_parameter<std::string>(
          "adxl375_topic", "/sensor_msgs/msg/Imu")),
      adxl375I2cDev(this->declare_parameter<std::string>("adxl375_i2c_dev",
                                                         "/dev/i2c-1")),
      adxl375Addr(static_cast<uint8_t>(
          this->declare_parameter<int>("adxl375_addr", 0x53))),
      ina260Topic(this->declare_parameter<std::string>(
          "ina260_topic", "/sensor_msgs/msg/BatteryState")),
      ina260I2cDev(
          this->declare_parameter<std::string>("ina260_i2c_dev", "/dev/i2c-1")),
      ina260Addr(static_cast<uint8_t>(
          this->declare_parameter<int>("ina260_addr", 0x40))),
      adxl375(adxl375I2cDev, adxl375Addr),
      ina260(ina260I2cDev, ina260Addr, Ina260::Config{}) {
  adxl375Publisher =
      this->create_publisher<sensor_msgs::msg::Imu>(adxl375Topic, 10);
  ina260Publisher =
      this->create_publisher<sensor_msgs::msg::BatteryState>(ina260Topic, 10);

  timer = this->create_wall_timer(
      std::chrono::milliseconds(readIntervalMs),
      std::bind(&SensorReaderNode::timeoutCallback, this));

  adxl375Initialized = adxl375.init();
  ina260Initialized = ina260.init();

  if (adxl375Initialized) {
    RCLCPP_INFO(this->get_logger(), "ADXL375 initialized successfully.");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize ADXL375.");
  }

  if (ina260Initialized) {
    RCLCPP_INFO(this->get_logger(), "INA260 initialized successfully.");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize INA260.");
  }
}

SensorReaderNode::~SensorReaderNode() = default;

void SensorReaderNode::timeoutCallback() {
  auto imuMsg = sensor_msgs::msg::Imu();
  imuMsg.header.stamp = this->now();

  auto batteryMsg = sensor_msgs::msg::BatteryState();
  batteryMsg.header.stamp = this->now();

  if (adxl375Initialized) {
    auto accelMS2 = adxl375.readMS2();
    imuMsg.header.frame_id = "adxl375";

    if (accelMS2.has_value()) {

      imuMsg.linear_acceleration.x = accelMS2->x;
      imuMsg.linear_acceleration.y = accelMS2->y;
      imuMsg.linear_acceleration.z = accelMS2->z;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to read data from ADXL375.");
      imuMsg.linear_acceleration.x = NAN;
      imuMsg.linear_acceleration.y = NAN;
      imuMsg.linear_acceleration.z = NAN;
    }
  } else {
    imuMsg.linear_acceleration.x = NAN;
    imuMsg.linear_acceleration.y = NAN;
    imuMsg.linear_acceleration.z = NAN;
  }

  adxl375Publisher->publish(imuMsg);

  if (ina260Initialized) {
    auto voltageOpt = ina260.readBusVoltage_V();
    auto currentOpt = ina260.readCurrent_A();

    batteryMsg.header.frame_id = "ina260";

    if (voltageOpt.has_value() && currentOpt.has_value()) {
      batteryMsg.voltage = static_cast<float>(voltageOpt.value());
      batteryMsg.current = static_cast<float>(currentOpt.value());
      ina260Publisher->publish(batteryMsg);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to read data from INA260.");
      batteryMsg.voltage = NAN;
      batteryMsg.current = NAN;
    }
  } else {
    batteryMsg.voltage = NAN;
    batteryMsg.current = NAN;
  }

  ina260Publisher->publish(batteryMsg);
}
