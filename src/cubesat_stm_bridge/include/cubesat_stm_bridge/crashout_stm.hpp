#pragma once
#include "stm_data.hpp"
#include <array>
#include <cstdint>
#include <exception>
#include <optional>
#include <string>

namespace StmBridge {

constexpr std::size_t TRANSFER_SIZE = 8;
using Transfer = std::array<uint8_t, TRANSFER_SIZE>;

class SpiException : std::exception {};

class CrashoutSTM {
  public:
    CrashoutSTM() = default;
    CrashoutSTM(std::string spidev, uint32_t speed_hz);
    ~CrashoutSTM();

    bool open(std::string spidev, uint32_t speed_hz);

    void recover();
    std::optional<Status> getStatus();
    std::optional<ArmPose> getArmPoseEst();
    std::optional<ArmPose> getArmTarget();
    std::optional<ServoTargets> getServoTargets();
    std::optional<FlipServoMotion> getServoMotion(FlipServo servoid);
    std::optional<Vec3_16> getBaseIMU();
    std::optional<Vec3_16> getLink1IMU();
    std::optional<Vec3_16> getLink2IMU();

    void reset();
    void hard_reset();
    void startHold();
    void setPoseEst(const ArmPose &pose);
    void setArmTarget(const ArmPose &pose);
    void setJogMovement(uint8_t motor, uint16_t iterations, int16_t mv);
    void startArmMovement(bool allow_overcurrent);

    void startJogMovement();
    void startServoMovement(FlipServo servoid);
    void stopMovement();
    void setServoMotion(FlipServo servoid, FlipServoMotion motion);
    
    std::optional<Status> setBaseImuAndReturnStatus(const Vec3_16 &base);

  private:
    std::optional<Transfer> transceive(const Transfer &outbound);

    int spi_fd = -1;
    uint32_t speed_hz = 4000000;
};

} // namespace StmBridge