#include "cubesat_stm_bridge/crashout_stm.hpp"
#include <cstdio>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <optional>
#include <span>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>

namespace StmBridge {
constexpr std::chrono::duration reset_time{std::chrono::milliseconds(100)};
constexpr std::chrono::duration between_time{std::chrono::milliseconds(2)};

void dump_transfer(std::string prefix, Transfer xfer) {
    printf("%s ", prefix.c_str());
    for (size_t i = 0; i < TRANSFER_SIZE; i++) {
        printf("%02x ", xfer[i]);
    }
    printf("\n");
}

bool CheckStatusBit(StatusWord word, StatusBit bit_index){
    return (word >> bit_index) & 1;
}

enum class SpiCommand : uint8_t {
    NoOp = 0, // do nothing, just asking for status
    Reset = 1,
    R_ReadLink1Accel = 2,
    R_ReadLink2Accel = 3,
    R_ReadBaseAccel = 4,
    WriteBaseAccel = 5,
    StartArm = 6,
    StartServo1 = 7,
    StartServo2 = 8,
    StartServo3 = 9,
    StopMoving = 10,

    WritePoseEst = 11,  // 'rezero' yaw, spitch, epitch, wpitch
    R_ReadPoseEst = 12, // yaw, spitch, epitch, wpitch

    WriteArmTarget = 13,
    R_ReadArmTarget = 14,

    WriteFlipServo1Motion = 15,
    WriteFlipServo2Motion = 16,
    WriteFlipServo3Motion = 17,

    R_ReadFlipServo1Motion = 18,
    R_ReadFlipServo2Motion = 19,
    R_ReadFlipServo3Motion = 20,

    R_ReadTemps = 21, // stm, link1, link2
};

bool CrashoutSTM::open(std::string spidev, uint32_t speed_hz) {
    spi_fd = open(spidev.c_str(), O_RDWR | O_CLOEXEC);
    if (spi_fd < 0) {
        return false;
    }

    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) != 0 || ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) != 0 ||
        ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) != 0) {
        return false;
    }
    return true;
}

CrashoutSTM::~CrashoutSTM() {
    if (spi_fd > 0) {
        close(spi_fd);
        spi_fd = -1;
    }
}

ArmPose decode_pose(const std::span<uint8_t> &buf) {
    return {
        static_cast<int8_t>(buf[0]),
        static_cast<int8_t>(buf[1]),
        static_cast<int8_t>(buf[2]),
        static_cast<int8_t>(buf[3]),
    };
}

FlipServoMotion decode_servo_motion(const std::span<uint8_t> &buf) {
    return FlipServoMotion{
        .open_duration = buf[0],
        .openness = buf[1],
        .open_travel_duration = buf[2],
        .closedness = buf[3],
        .close_travel_duration = buf[4],
    };
}

void CrashoutSTM::startArmMovement() {
    uint8_t cmd = (uint8_t)SpiCommand::StartArm;
    transceive(Transfer{cmd});
}

void CrashoutSTM::startServoMovement(FlipServo servoid) {
    uint8_t cmd = (uint8_t)((uint8_t)SpiCommand::StartServo1 + (uint8_t)servoid);
    transceive(Transfer{cmd});
}
void CrashoutSTM::setServoMotion(FlipServo servoid, FlipServoMotion motion) {
    uint8_t cmd = (uint8_t)SpiCommand::WriteFlipServo1Motion + servoid;
    Transfer outbound{cmd,
                      motion.open_duration,
                      motion.openness,
                      motion.open_travel_duration,
                      motion.closedness,
                      motion.close_travel_duration};
    dump_transfer("Outbound: ", outbound);
    transceive(outbound);
}

void CrashoutSTM::setArmTarget(const ArmPose &pose) {
    uint8_t cmd = (uint8_t)SpiCommand::WriteArmTarget;
    Transfer outbound{cmd, static_cast<uint8_t>(pose.shoulder_yaw), static_cast<uint8_t>(pose.shoulder_pitch),
                      static_cast<uint8_t>(pose.elbow_pitch), static_cast<uint8_t>(pose.wrist_pitch)};

    dump_transfer("Outbound: ", outbound);
    transceive(outbound);
}

void CrashoutSTM::setPoseEst(const ArmPose &pose) {
    uint8_t cmd = (uint8_t)SpiCommand::WritePoseEst;
    Transfer outbound{cmd, static_cast<uint8_t>(pose.shoulder_yaw), static_cast<uint8_t>(pose.shoulder_pitch),
                      static_cast<uint8_t>(pose.elbow_pitch), static_cast<uint8_t>(pose.wrist_pitch)};

    dump_transfer("Outbound: ", outbound);
    transceive(outbound);
}

std::optional<FlipServoMotion> CrashoutSTM::getServoMotion(FlipServo servoid) {
    // ask
    uint8_t cmd = (uint8_t)SpiCommand::R_ReadFlipServo1Motion + (uint8_t)servoid;
    transceive(Transfer{cmd});
    // listen
    std::optional<Transfer> xfer = transceive(Transfer{});
    if (!xfer.has_value()) {
        return std::nullopt;
    }
    dump_transfer("servo back: ", *xfer);
    return decode_servo_motion(std::span<uint8_t>{*xfer}.subspan(2, 7));
}

void CrashoutSTM::reset() {
    uint8_t b = (uint8_t)SpiCommand::Reset;
    Transfer outbuf{b, b, b, b, b, b, b, b};
    transceive(outbuf); // enough 1s and one will count as a reset
    transceive(outbuf);
    std::this_thread::sleep_for(reset_time);
    recover();
}

std::optional<Status> CrashoutSTM::getStatus() {
    // a no op will have the default command come out of the stm which is the
    // status
    Transfer outbound{(uint8_t)SpiCommand::NoOp, 0, 0, 0, 0, 0, 0, 0};
    std::optional<Transfer> resp = transceive(outbound);
    if (!resp.has_value()) {
        return {};
    }
    Status status{};
    status.status_word = (resp->at(0) << 8) | resp->at(1);
    status.pose = decode_pose(std::span<uint8_t>{*resp}.subspan(2, 6));
    status.uptime_lsw = (resp->at(6) << 8) | resp->at(7);
    return status;
}

void dumpStatus(StatusWord word) {
    auto testBit = [=](int bit) { return (word >> bit) & 1; };
    auto testBitAndPrintChar = [&](int bit, char c) {
        if (testBit(bit)) {
            printf("%c", c);
        } else {
            printf(" ");
        }
    };

    uint8_t rtype = (word >> StatusBit_RType0);
    printf("%d:", rtype);
    testBitAndPrintChar(StatusBit_Overtemp, 'o');
    testBitAndPrintChar(StatusBit_MotorEn, 'M');
    testBitAndPrintChar(StatusBit_FlipServoEn, '^');
    testBitAndPrintChar(StatusBit_WristServoEn, 'W');
    testBitAndPrintChar(StatusBit_MovingArmFailed, '!');
    testBitAndPrintChar(StatusBit_MovingFlipServo3, '3');
    testBitAndPrintChar(StatusBit_MovingFlipServo2, '2');
    testBitAndPrintChar(StatusBit_MovingFlipServo1, '1');
    testBitAndPrintChar(StatusBit_MovingArm, 'A');
    testBitAndPrintChar(StatusBit_Booted, 'B');
}

void dumpStatusLn(StatusWord word) {
    dumpStatus(word);
    printf("\n");
}

std::optional<ArmPose> CrashoutSTM::getArmPoseEst() {
    Transfer outbound{(uint8_t)SpiCommand::R_ReadPoseEst};
    transceive(outbound);
    std::optional<Transfer> response = transceive(Transfer{});
    if (!response.has_value()) {
        return std::nullopt;
    }
    // 0 - 2 is status bits and response kind
    // TODO check response kind
    return decode_pose(std::span<uint8_t>{*response}.subspan(2, 6));
}
std::optional<ArmPose> CrashoutSTM::getArmTarget() {
    Transfer outbound{(uint8_t)SpiCommand::R_ReadArmTarget};
    transceive(outbound);
    std::optional<Transfer> response = transceive(Transfer{});
    if (!response.has_value()) {
        return std::nullopt;
    }
    // 0 - 2 is status bits and response kind
    // TODO check response kind
    return decode_pose(std::span<uint8_t>{*response}.subspan(2, 6));
}

void CrashoutSTM::recover() {
    // clear any waiting response by getting and discarding it
    transceive(Transfer{});
}

std::optional<Transfer> CrashoutSTM::transceive(const Transfer &outbound) {
    Transfer inbound{0};
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)outbound.data(),
        .rx_buf = (unsigned long)inbound.data(),
        .len = TRANSFER_SIZE,
        .speed_hz = speed_hz,
        .delay_usecs = 5, // TODO check
        .bits_per_word = 8,
        .cs_change = 0,
        .tx_nbits = 0,
        .rx_nbits = 0,
        .word_delay_usecs = 5,
        .pad = 0,
    };
    int res = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    if (res == -1) {
        return std::nullopt;
    }
    std::this_thread::sleep_for(between_time);

    return inbound;
}
} // namespace StmBridge