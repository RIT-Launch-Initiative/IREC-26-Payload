/*
 * RadioTransceiverNode bridges ROS topics to the SX126x radio.
 */

#pragma once

#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <vector>
#include <map>

#include <gpiod.h>
#include <rclcpp/rclcpp.hpp>

#include "arm_msgs/msg/arm_command.hpp"
#include "arm_msgs/msg/arm_status.hpp"
#include "arm_msgs/msg/heartbeat_status.hpp"
#include "arm_msgs/msg/radio_parameters_command.hpp"
#include "arm_msgs/msg/image_request.hpp"
#include "arm_msgs/msg/image_data.hpp"

extern "C" {
#include "sx126x.h"
#include "sx126x_hal.h"
}

class RadioTransceiverNode : public rclcpp::Node {
public:
	RadioTransceiverNode();
	~RadioTransceiverNode() override;

private:
	struct RadioConfig {
		uint32_t frequencyHz;
		int8_t txPowerDbm;
		uint8_t spreadingFactor;
		uint32_t bandwidthHz;
		uint32_t codingRate;
		uint16_t preambleLength;
	};

	struct HalContext {
		int spiFd;
		gpiod_chip *chip;
		gpiod_line *reset;
		gpiod_line *busy;
	};

	uint16_t txPort{9000};
	uint16_t rxMoveArmPort{9100};
	uint16_t rxSetRadioParamsPort{9101};
	uint16_t rxRequestImagePort{9102};
	uint16_t txImageDataPort{9200};
	static constexpr uint8_t kMsgHeartbeat = 0x01;
	static constexpr uint8_t kMsgArmTelemetry = 0x02;
	static constexpr uint8_t kMsgImageData = 0x03;
	static constexpr uint8_t kMsgMoveArm = 0x10;
	static constexpr uint8_t kMsgSetRadioParams = 0x11;
	static constexpr uint8_t kMsgRequestImage = 0x12;

	void heartbeatCallback(const arm_msgs::msg::HeartbeatStatus::SharedPtr msg);
	void armStatusCallback(const arm_msgs::msg::ArmStatus::SharedPtr msg);
	void armCommandCallback(const arm_msgs::msg::ArmCommand::SharedPtr msg);
	void imageDataCallback(const arm_msgs::msg::ImageData::SharedPtr msg);
	void txTimerCallback();
	void rxPollCallback();

	bool initHardware();
	bool configureRadio(const RadioConfig &config);
	bool applyRadioConfig(const RadioConfig &config);
	bool restartRx();
	bool transmitFrame(const std::vector<uint8_t> &payload);
	void pollRadioIrqs();
	void handleInboundFrame(const std::vector<uint8_t> &frame);
	bool decodeFrame(const std::vector<uint8_t> &frame, uint16_t &port,
									 std::vector<uint8_t> &payload) const;
	std::vector<uint8_t> buildFrame(const std::vector<uint8_t> &payload) const;

	void handleMoveArmCommand(const std::vector<uint8_t> &payload);
	void handleSetRadioParams(const std::vector<uint8_t> &payload);
	void handleRequestImage(const std::vector<uint8_t> &payload);
	std::vector<uint8_t> serializeHeartbeat() const;
	std::vector<uint8_t> serializeArmTelemetry() const;
	std::vector<uint8_t> serializeImageData(const arm_msgs::msg::ImageData &imageData) const;

	sx126x_lora_bw_t toLoraBandwidth(uint32_t bandwidth) const;
	sx126x_lora_cr_t toLoraCodingRate(uint32_t codingRate) const;
	uint8_t computeLdro(uint8_t sf, uint32_t bandwidthHz) const;
	bool configsEqual(const RadioConfig &a, const RadioConfig &b) const;

	void scheduleRevertFromGsCommand(uint8_t rxTests, uint8_t txTests);
	void maybeRevertRadioConfig();

	void writeUint16(std::vector<uint8_t> &out, uint16_t value) const;
	void writeUint32(std::vector<uint8_t> &out, uint32_t value) const;
	void writeFloat(std::vector<uint8_t> &out, float value) const;
	uint16_t readUint16(const std::vector<uint8_t> &in, size_t offset) const;
	int16_t readInt16(const std::vector<uint8_t> &in, size_t offset) const;
	uint32_t readUint32(const std::vector<uint8_t> &in, size_t offset) const;

	std::string spiDevice;
	std::string gpioChipName;
	int resetGpioLine;
	int busyGpioLine;
	uint32_t spiSpeedHz;
	int txIntervalMs;
	int rxPollIntervalMs;
	int txTimeoutMs;
	int gsTimeoutSec;
	int radioRevertSec;


    // State
	RadioConfig defaultConfig{};
	RadioConfig activeConfig{};
	HalContext halContext{};
	std::mutex radioMutex;
	bool radioReady{false};

	rclcpp::Time lastInboundTime{};
	std::optional<rclcpp::Time> revertDeadline;

	arm_msgs::msg::HeartbeatStatus lastHeartbeat{};
	arm_msgs::msg::ArmStatus lastArmStatus{};
	arm_msgs::msg::ArmCommand lastArmCommand{};
	bool haveHeartbeat{false};
	bool haveArmStatus{false};

	// Image transmission state
	std::map<uint8_t, std::vector<arm_msgs::msg::ImageData>> pendingImageChunks;
	std::mutex imageDataMutex;

    // Pub Subs
	rclcpp::Subscription<arm_msgs::msg::HeartbeatStatus>::SharedPtr heartbeatSub;
	rclcpp::Subscription<arm_msgs::msg::ArmStatus>::SharedPtr armStatusSub;
	rclcpp::Subscription<arm_msgs::msg::ArmCommand>::SharedPtr armCommandSub;
	rclcpp::Subscription<arm_msgs::msg::ImageData>::SharedPtr imageDataSub;
	rclcpp::Publisher<arm_msgs::msg::ArmCommand>::SharedPtr armCommandPub;
	rclcpp::Publisher<arm_msgs::msg::ImageRequest>::SharedPtr imageRequestPub;
	rclcpp::TimerBase::SharedPtr txTimer;
	rclcpp::TimerBase::SharedPtr rxPollTimer;
};
