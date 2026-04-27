#include "cubesat_pi_io/pi_io_node.hpp"

#include <chrono>

namespace cubesat_pi_io {

namespace {

std::chrono::nanoseconds periodFromHz(double hz) {
  if (hz <= 0.0) {
    return std::chrono::seconds(1);
  }
  return std::chrono::nanoseconds(static_cast<int64_t>(1e9 / hz));
}

} // namespace

PiIoNode::PiIoNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("pi_io_node", options) {
  // Shared (consumed by other nodes; declared here so launch can pass it
  // freely).
  declare_parameter<std::string>("flight_dir", "");

  // GPS
  const auto gps_dev =
      declare_parameter<std::string>("gps.uart_device", "/dev/serial0");
  const auto gps_baud = declare_parameter<int>("gps.baud_rate", 9600);
  const auto gps_hz = declare_parameter<double>("gps.poll_hz", 5.0);

  // INA260
  const auto ina_dev =
      declare_parameter<std::string>("ina260.i2c_device", "/dev/i2c-1");
  const auto ina_addr = declare_parameter<int>("ina260.address", 0x40);
  const auto ina_hz = declare_parameter<double>("ina260.poll_hz", 5.0);

  // LIS3DH
  const auto lis_dev =
      declare_parameter<std::string>("lis3dh.i2c_device", "/dev/i2c-1");
  const auto lis_addr = declare_parameter<int>("lis3dh.address", 0x18);
  const auto lis_rate = declare_parameter<int>("lis3dh.sample_rate_hz", 100);
  const auto lis_range = declare_parameter<int>("lis3dh.range_g", 16);
  const auto lis_hz = declare_parameter<double>("lis3dh.poll_hz", 50.0);

  gps_pub = create_publisher<cubesat_msgs::msg::GpsSample>("/pi/gps", 10);
  power_pub =
      create_publisher<cubesat_msgs::msg::PowerSample>("/pi/power", 10);
  accel_pub =
      create_publisher<cubesat_msgs::msg::AccelSample>("/pi/lis3dh", 50);

  gps.open(gps_dev, gps_baud);
  ina.open(ina_dev, static_cast<uint8_t>(ina_addr));
  if (lis.open(lis_dev, static_cast<uint8_t>(lis_addr))) {
    lis.configure(static_cast<uint16_t>(lis_rate),
                   static_cast<uint8_t>(lis_range));
  }

  gps_timer =
      create_wall_timer(periodFromHz(gps_hz), [this] { onGpsTimer(); });
  power_timer =
      create_wall_timer(periodFromHz(ina_hz), [this] { onPowerTimer(); });
  accel_timer =
      create_wall_timer(periodFromHz(lis_hz), [this] { onAccelTimer(); });
}

void PiIoNode::onGpsTimer() {
  auto fix = gps.readFix();
  if (!fix) {
    return;
  }
  cubesat_msgs::msg::GpsSample msg;
  msg.stamp = now();
  msg.latitude = fix->latitude;
  msg.longitude = fix->longitude;
  msg.altitude_m = fix->altitude_m;
  msg.fix_type = fix->fix_type;
  msg.satellites = fix->satellites_visible;
  gps_pub->publish(msg);
}

void PiIoNode::onPowerTimer() {
  auto p = ina.read();
  if (!p) {
    return;
  }
  cubesat_msgs::msg::PowerSample msg;
  msg.stamp = now();
  msg.bus_voltage_v = p->bus_voltage_v;
  msg.current_a = p->current_a;
  msg.power_w = p->power_w;
  power_pub->publish(msg);
}

void PiIoNode::onAccelTimer() {
  auto a = lis.read();
  if (!a) {
    return;
  }
  cubesat_msgs::msg::AccelSample msg;
  msg.stamp = now();
  msg.ax = a->ax_mps2;
  msg.ay = a->ay_mps2;
  msg.az = a->az_mps2;
  accel_pub->publish(msg);
}

} // namespace cubesat_pi_io
