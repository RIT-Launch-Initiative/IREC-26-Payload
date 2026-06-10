#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <cubesat_msgs/msg/accel_sample.hpp>
#include <cubesat_msgs/msg/arm_status.hpp>
#include <cubesat_msgs/msg/flight_state.hpp>
#include <cubesat_msgs/msg/gps_sample.hpp>
#include <cubesat_msgs/msg/power_sample.hpp>

#include "cubesat_flight_logger/flight_logger.hpp"

namespace cubesat_flight_logger {

// arm status bits in FlightLogRecord.arm_status_flags
enum ArmFlagBit : uint16_t {
    ArmFlag_Booted = 1 << 0,
    ArmFlag_MoveFailed = 1 << 1,
    ArmFlag_WristServoEn = 1 << 2,
    ArmFlag_FlipServoEn = 1 << 3,
    ArmFlag_MotorEn = 1 << 4,
    ArmFlag_CantTrustImuLink = 1 << 5,
    ArmFlag_EncodersNotUpdating = 1 << 6,
    // bits 8..11: arm state machine state
};

class FlightLoggerNode : public rclcpp::Node {
  public:
    explicit FlightLoggerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : rclcpp::Node("flight_logger_node", options) {
        const auto flight_dir = declare_parameter<std::string>("flight_dir", "");
        const auto file_name = declare_parameter<std::string>("log_file_name", "flight_log.bin");
        const auto fsync_every = declare_parameter<int64_t>("fsync_every_records", 50);

        std::string path = flight_dir.empty() ? ("/tmp/" + file_name) : (flight_dir + "/" + file_name);
        if (!logger.open(path, (size_t)fsync_every)) {
            RCLCPP_ERROR(get_logger(), "Could not open flight log at %s: nothing will be recorded", path.c_str());
        } else {
            RCLCPP_INFO(get_logger(), "Logging flight data to %s (fsync every %ld records)", path.c_str(),
                        fsync_every);
        }

        // one record per accel sample (50 Hz from pi_io); the other topics just
        // refresh the latest-known values folded into each record
        accel_sub = create_subscription<cubesat_msgs::msg::AccelSample>(
            "pi/lis3dh", 50, [this](const cubesat_msgs::msg::AccelSample::SharedPtr msg) { onAccel(*msg); });
        gps_sub = create_subscription<cubesat_msgs::msg::GpsSample>(
            "pi/gps", 10, [this](const cubesat_msgs::msg::GpsSample::SharedPtr msg) { last_gps = *msg; });
        power_sub = create_subscription<cubesat_msgs::msg::PowerSample>(
            "pi/power", 10, [this](const cubesat_msgs::msg::PowerSample::SharedPtr msg) { last_power = *msg; });
        state_sub = create_subscription<cubesat_msgs::msg::FlightState>(
            "pi/flight_state", 10, [this](const cubesat_msgs::msg::FlightState::SharedPtr msg) { last_state = *msg; });
        arm_sub = create_subscription<cubesat_msgs::msg::ArmStatus>(
            "stm/arm_status", 10, [this](const cubesat_msgs::msg::ArmStatus::SharedPtr msg) { last_arm = *msg; });

        // periodic flush so a quiet system (no accel samples => no buffer fill)
        // still gets its tail written out
        flush_timer = create_wall_timer(std::chrono::seconds(5), [this] { logger.flush(); });
    }

    ~FlightLoggerNode() override {
        // rclcpp's default signal handler turns SIGTERM/SIGINT into shutdown,
        // spin returns, and this destructor flushes + fsyncs the tail
        logger.close();
    }

  private:
    void onAccel(const cubesat_msgs::msg::AccelSample &accel) {
        FlightLogRecord rec{};
        rec.timestamp_ns = (uint64_t)now().nanoseconds();
        rec.accel_x_mps2 = accel.ax;
        rec.accel_y_mps2 = accel.ay;
        rec.accel_z_mps2 = accel.az;
        rec.latitude = last_gps.latitude;
        rec.longitude = last_gps.longitude;
        rec.altitude_m = last_gps.altitude_m;
        rec.battery_v = last_power.bus_voltage_v;
        rec.flight_state = last_state.state;
        rec.arm_status_flags = packArmFlags();
        logger.log(rec);
    }

    uint16_t packArmFlags() const {
        uint16_t flags = 0;
        flags |= last_arm.booted ? ArmFlag_Booted : 0;
        flags |= last_arm.arm_move_failed ? ArmFlag_MoveFailed : 0;
        flags |= last_arm.wrist_servo_en ? ArmFlag_WristServoEn : 0;
        flags |= last_arm.flip_servo_en ? ArmFlag_FlipServoEn : 0;
        flags |= last_arm.motor_en ? ArmFlag_MotorEn : 0;
        flags |= last_arm.cant_trust_imu_link ? ArmFlag_CantTrustImuLink : 0;
        flags |= last_arm.encoders_not_updating ? ArmFlag_EncodersNotUpdating : 0;
        flags |= (uint16_t)((last_arm.state.state & 0x0F) << 8);
        return flags;
    }

    FlightLogger logger;

    cubesat_msgs::msg::GpsSample last_gps{};
    cubesat_msgs::msg::PowerSample last_power{};
    cubesat_msgs::msg::FlightState last_state{};
    cubesat_msgs::msg::ArmStatus last_arm{};

    rclcpp::Subscription<cubesat_msgs::msg::AccelSample>::SharedPtr accel_sub;
    rclcpp::Subscription<cubesat_msgs::msg::GpsSample>::SharedPtr gps_sub;
    rclcpp::Subscription<cubesat_msgs::msg::PowerSample>::SharedPtr power_sub;
    rclcpp::Subscription<cubesat_msgs::msg::FlightState>::SharedPtr state_sub;
    rclcpp::Subscription<cubesat_msgs::msg::ArmStatus>::SharedPtr arm_sub;

    rclcpp::TimerBase::SharedPtr flush_timer;
};

} // namespace cubesat_flight_logger

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cubesat_flight_logger::FlightLoggerNode>());
    rclcpp::shutdown();
    return 0;
}
