#include "radio_transceiver/RadioTransceiverNode.hpp"

#include <algorithm>
#include <cstring>
#include <thread>

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

using namespace std::chrono_literals;

RadioTransceiverNode::RadioTransceiverNode() : Node("radio_transceiver") {
	halContext.spiFd = -1;
	halContext.chip = nullptr;
	halContext.reset = nullptr;
	halContext.busy = nullptr;

	spiDevice = this->declare_parameter<std::string>("spi_device", "/dev/spidev0.0");
	gpioChipName = this->declare_parameter<std::string>("gpio_chip_name", "gpiochip0");
	resetGpioLine = this->declare_parameter<int>("reset_gpio_line", 31);
	busyGpioLine = this->declare_parameter<int>("busy_gpio_line", 12);
	spiSpeedHz = static_cast<uint32_t>(this->declare_parameter<int>("spi_speed_hz", 1000000));
	txIntervalMs = this->declare_parameter<int>("tx_interval_ms", 5000);
	rxPollIntervalMs = this->declare_parameter<int>("rx_poll_interval_ms", 50);
	txTimeoutMs = this->declare_parameter<int>("tx_timeout_ms", 3000);
	gsTimeoutSec = this->declare_parameter<int>("gs_timeout_sec", 60);
	radioRevertSec = this->declare_parameter<int>("radio_revert_sec", 30);
	txPort = static_cast<uint16_t>(this->declare_parameter<int>("tx_port", 9000));
	rxMoveArmPort = static_cast<uint16_t>(this->declare_parameter<int>("rx_move_arm_port", 9100));
	rxSetRadioParamsPort = static_cast<uint16_t>(this->declare_parameter<int>("rx_set_radio_params_port", 9101));
	rxRequestImagePort = static_cast<uint16_t>(this->declare_parameter<int>("rx_request_image_port", 9102));
	txImageDataPort = static_cast<uint16_t>(this->declare_parameter<int>("tx_image_data_port", 9200));

	defaultConfig.frequencyHz = static_cast<uint32_t>(this->declare_parameter<int>("frequency_hz", 915000000));
	defaultConfig.txPowerDbm = static_cast<int8_t>(this->declare_parameter<int>("tx_power_dbm", 22));
	defaultConfig.spreadingFactor = static_cast<uint8_t>(this->declare_parameter<int>("spreading_factor", 12));
	defaultConfig.bandwidthHz = static_cast<uint32_t>(this->declare_parameter<int>("bandwidth_hz", 125000));
	defaultConfig.codingRate = static_cast<uint32_t>(this->declare_parameter<int>("coding_rate", 5));
	defaultConfig.preambleLength = static_cast<uint16_t>(this->declare_parameter<int>("preamble_length", 8));
	activeConfig = defaultConfig;

	const auto heartbeatTopic = this->declare_parameter<std::string>("heartbeat_topic", "heartbeat_status");
	const auto armStatusTopic = this->declare_parameter<std::string>("arm_status_topic", "arm_status");
	const auto armCommandTopic = this->declare_parameter<std::string>("arm_command_topic", "arm_command");
	const auto armTargetTopic = this->declare_parameter<std::string>("arm_target_topic", "/arm_target");
	const auto imageDataTopic = this->declare_parameter<std::string>("image_data_topic", "image_data");
	const auto imageRequestTopic = this->declare_parameter<std::string>("image_request_topic", "image_request");

	heartbeatSub = this->create_subscription<arm_msgs::msg::HeartbeatStatus>(
			heartbeatTopic, 10,
			std::bind(&RadioTransceiverNode::heartbeatCallback, this, std::placeholders::_1));

	armStatusSub = this->create_subscription<arm_msgs::msg::ArmStatus>(
			armStatusTopic, 10,
			std::bind(&RadioTransceiverNode::armStatusCallback, this, std::placeholders::_1));

	armCommandSub = this->create_subscription<arm_msgs::msg::ArmCommand>(
			armCommandTopic, 10,
			std::bind(&RadioTransceiverNode::armCommandCallback, this, std::placeholders::_1));

	imageDataSub = this->create_subscription<arm_msgs::msg::ImageData>(
			imageDataTopic, 10,
			std::bind(&RadioTransceiverNode::imageDataCallback, this, std::placeholders::_1));

	armCommandPub = this->create_publisher<arm_msgs::msg::ArmCommand>(armTargetTopic, 10);
	imageRequestPub = this->create_publisher<arm_msgs::msg::ImageRequest>(imageRequestTopic, 10);

	radioReady = initHardware();
	if (!radioReady) {
		RCLCPP_ERROR(this->get_logger(), "RadioTransceiverNode failed to initialize radio hardware");
	}

	txTimer = this->create_wall_timer(
			std::chrono::milliseconds(txIntervalMs),
			std::bind(&RadioTransceiverNode::txTimerCallback, this));
	rxPollTimer = this->create_wall_timer(
			std::chrono::milliseconds(rxPollIntervalMs),
			std::bind(&RadioTransceiverNode::rxPollCallback, this));

	RCLCPP_INFO(this->get_logger(), "RadioTransceiverNode started");
}

RadioTransceiverNode::~RadioTransceiverNode() {
	if (halContext.reset) {
		gpiod_line_release(halContext.reset);
	}
	if (halContext.busy) {
		gpiod_line_release(halContext.busy);
	}
	if (halContext.chip) {
		gpiod_chip_close(halContext.chip);
	}
	if (halContext.spiFd >= 0) {
		close(halContext.spiFd);
	}
}

void RadioTransceiverNode::heartbeatCallback(
		const arm_msgs::msg::HeartbeatStatus::SharedPtr msg) {
	lastHeartbeat = *msg;
	haveHeartbeat = true;
}

void RadioTransceiverNode::armStatusCallback(
		const arm_msgs::msg::ArmStatus::SharedPtr msg) {
	lastArmStatus = *msg;
	haveArmStatus = true;
}

void RadioTransceiverNode::armCommandCallback(
		const arm_msgs::msg::ArmCommand::SharedPtr msg) {
	lastArmCommand = *msg;
}

void RadioTransceiverNode::imageDataCallback(
		const arm_msgs::msg::ImageData::SharedPtr msg) {
	std::lock_guard<std::mutex> lock(imageDataMutex);
	
	// Store the chunk
	pendingImageChunks[msg->image_id].push_back(*msg);
	
	RCLCPP_DEBUG(this->get_logger(), "Received image chunk %u/%u for image ID %u",
	             msg->chunk_index + 1, msg->total_chunks, msg->image_id);
	
	// Check if we have all chunks for this image
	auto &chunks = pendingImageChunks[msg->image_id];
	if (chunks.size() == msg->total_chunks) {
		RCLCPP_INFO(this->get_logger(), "All chunks received for image ID %u, ready for transmission",
		            msg->image_id);
	}
}

bool RadioTransceiverNode::initHardware() {
	halContext.spiFd = open(spiDevice.c_str(), O_RDWR);
	if (halContext.spiFd < 0) {
		RCLCPP_ERROR(this->get_logger(), "Failed to open SPI device: %s", spiDevice.c_str());
		return false;
	}

	uint8_t mode = SPI_MODE_0;
	uint8_t bits = 8;
	if (ioctl(halContext.spiFd, SPI_IOC_WR_MODE, &mode) != 0 ||
			ioctl(halContext.spiFd, SPI_IOC_WR_BITS_PER_WORD, &bits) != 0 ||
			ioctl(halContext.spiFd, SPI_IOC_WR_MAX_SPEED_HZ, &spiSpeedHz) != 0) {
		RCLCPP_ERROR(this->get_logger(), "Failed to configure SPI parameters");
		return false;
	}

	halContext.chip = gpiod_chip_open_by_name(gpioChipName.c_str());
	if (!halContext.chip) {
		RCLCPP_ERROR(this->get_logger(), "Failed to open gpio chip: %s", gpioChipName.c_str());
		return false;
	}

	halContext.reset = gpiod_chip_get_line(halContext.chip, resetGpioLine);
	if (!halContext.reset ||
			gpiod_line_request_output(halContext.reset, "sx126x_reset", 1) != 0) {
		RCLCPP_ERROR(this->get_logger(), "Failed to request reset GPIO line %d", resetGpioLine);
		return false;
	}

	if (busyGpioLine >= 0) {
		halContext.busy = gpiod_chip_get_line(halContext.chip, busyGpioLine);
		if (!halContext.busy ||
				gpiod_line_request_input(halContext.busy, "sx126x_busy") != 0) {
			RCLCPP_WARN(this->get_logger(), "Busy GPIO line %d unavailable; continuing without busy", busyGpioLine);
			halContext.busy = nullptr;
		}
	}

	if (sx126x_hal_reset(&halContext) != SX126X_HAL_STATUS_OK) {
		RCLCPP_ERROR(this->get_logger(), "Radio reset failed");
		return false;
	}

	if (sx126x_hal_wakeup(&halContext) != SX126X_HAL_STATUS_OK) {
		RCLCPP_ERROR(this->get_logger(), "Radio wakeup failed");
		return false;
	}

	return configureRadio(activeConfig);
}

bool RadioTransceiverNode::applyRadioConfig(const RadioConfig &config) {
	bool ok = configureRadio(config);
	if (ok) {
		activeConfig = config;
		RCLCPP_INFO(this->get_logger(),
								"Applied radio config: freq=%u Hz, sf=%u, bw=%u Hz, cr=%u, tx=%d dBm",
								activeConfig.frequencyHz, activeConfig.spreadingFactor,
								activeConfig.bandwidthHz, activeConfig.codingRate,
								activeConfig.txPowerDbm);
	}
	return ok;
}

bool RadioTransceiverNode::configureRadio(const RadioConfig &config) {
	std::lock_guard<std::mutex> lock(radioMutex);

	if (sx126x_set_standby(&halContext, SX126X_STANDBY_CFG_RC) != SX126X_STATUS_OK) {
		RCLCPP_ERROR(this->get_logger(), "Failed to set standby");
		return false;
	}

	if (sx126x_set_pkt_type(&halContext, SX126X_PKT_TYPE_LORA) != SX126X_STATUS_OK) {
		RCLCPP_ERROR(this->get_logger(), "Failed to set packet type");
		return false;
	}

	sx126x_pa_cfg_params_t paConfig{.pa_duty_cycle = 0x04, .hp_max = 0x07, .device_sel = 0x00, .pa_lut = 0x01};
	sx126x_set_pa_cfg(&halContext, &paConfig);
	sx126x_set_tx_params(&halContext, config.txPowerDbm, SX126X_RAMP_200_US);

	auto bwEnum = toLoraBandwidth(config.bandwidthHz);
	auto crEnum = toLoraCodingRate(config.codingRate);
	auto sfEnum = static_cast<sx126x_lora_sf_t>(config.spreadingFactor);
	sx126x_mod_params_lora_t modParams{sfEnum, bwEnum, crEnum, computeLdro(config.spreadingFactor, config.bandwidthHz)};
	if (sx126x_set_lora_mod_params(&halContext, &modParams) != SX126X_STATUS_OK) {
		RCLCPP_ERROR(this->get_logger(), "Failed to set modulation params");
		return false;
	}

	sx126x_pkt_params_lora_t pktParams{config.preambleLength, SX126X_LORA_PKT_EXPLICIT, 0xFF, true, false};
	if (sx126x_set_lora_pkt_params(&halContext, &pktParams) != SX126X_STATUS_OK) {
		RCLCPP_ERROR(this->get_logger(), "Failed to set packet params");
		return false;
	}

	if (sx126x_set_buffer_base_address(&halContext, 0x00, 0x00) != SX126X_STATUS_OK) {
		RCLCPP_ERROR(this->get_logger(), "Failed to set buffer base");
		return false;
	}

	if (sx126x_set_rf_freq(&halContext, config.frequencyHz) != SX126X_STATUS_OK) {
		RCLCPP_ERROR(this->get_logger(), "Failed to set RF freq");
		return false;
	}

	sx126x_set_dio_irq_params(&halContext,
							SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_CRC_ERROR | SX126X_IRQ_TIMEOUT,
							SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_CRC_ERROR | SX126X_IRQ_TIMEOUT,
							0, 0);
	sx126x_clear_irq_status(&halContext, SX126X_IRQ_ALL);

	return restartRx();
}

bool RadioTransceiverNode::restartRx() {
	if (sx126x_set_rx(&halContext, SX126X_RX_CONTINUOUS) != SX126X_STATUS_OK) {
		RCLCPP_ERROR(this->get_logger(), "Failed to enter RX mode");
		return false;
	}
	return true;
}

void RadioTransceiverNode::txTimerCallback() {
	if (!radioReady) {
		return;
	}

	if (haveHeartbeat) {
		transmitFrame(serializeHeartbeat());
	}

	if (haveArmStatus) {
		transmitFrame(serializeArmTelemetry());
	}

	// Transmit pending image chunks
	{
		std::lock_guard<std::mutex> lock(imageDataMutex);
		for (auto &[imageId, chunks] : pendingImageChunks) {
			// Sort chunks by index
			std::sort(chunks.begin(), chunks.end(), 
			         [](const auto &a, const auto &b) { return a.chunk_index < b.chunk_index; });
			
			// Transmit each chunk
			for (const auto &chunk : chunks) {
				auto payload = serializeImageData(chunk);
				
				// Build frame with image data port
				std::vector<uint8_t> frame;
				frame.reserve(payload.size() + 2);
				frame.push_back(static_cast<uint8_t>(txImageDataPort & 0xFF));
				frame.push_back(static_cast<uint8_t>((txImageDataPort >> 8) & 0xFF));
				frame.insert(frame.end(), payload.begin(), payload.end());
				
				if (frame.size() <= 255) {
					transmitFrame(payload);
					RCLCPP_DEBUG(this->get_logger(), "Transmitted image chunk %u/%u for ID %u",
					            chunk.chunk_index + 1, chunk.total_chunks, imageId);
				} else {
					RCLCPP_WARN(this->get_logger(), "Image chunk too large: %zu bytes", frame.size());
				}
				
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}
		}
		// Clear transmitted chunks
		pendingImageChunks.clear();
	}

	maybeRevertRadioConfig();
}

void RadioTransceiverNode::rxPollCallback() {
	if (!radioReady) {
		return;
	}
	pollRadioIrqs();
}

bool RadioTransceiverNode::transmitFrame(const std::vector<uint8_t> &payload) {
	auto frame = buildFrame(payload);
	if (frame.size() > 255) {
		RCLCPP_WARN(this->get_logger(), "Frame too large (%zu bytes)", frame.size());
		return false;
	}

	std::lock_guard<std::mutex> lock(radioMutex);
	sx126x_pkt_params_lora_t pktParams{activeConfig.preambleLength, SX126X_LORA_PKT_EXPLICIT,
																		 static_cast<uint8_t>(frame.size()), true, false};
	sx126x_set_lora_pkt_params(&halContext, &pktParams);
	sx126x_set_buffer_base_address(&halContext, 0x00, 0x00);
	sx126x_write_buffer(&halContext, 0x00, frame.data(), static_cast<uint8_t>(frame.size()));
	sx126x_clear_irq_status(&halContext, SX126X_IRQ_ALL);
	sx126x_set_tx(&halContext, static_cast<uint32_t>(txTimeoutMs));

	bool sent = false;
	const int attempts = std::max(1, txTimeoutMs / 20);
	for (int i = 0; i < attempts; ++i) {
		sx126x_irq_mask_t irq = 0;
		sx126x_get_and_clear_irq_status(&halContext, &irq);
		if (irq & SX126X_IRQ_TX_DONE) {
			sent = true;
			break;
		}
		if (irq & SX126X_IRQ_TIMEOUT) {
			break;
		}
		std::this_thread::sleep_for(20ms);
	}

	restartRx();

	if (!sent) {
		RCLCPP_WARN(this->get_logger(), "TX did not complete before timeout");
	}

	return sent;
}

void RadioTransceiverNode::pollRadioIrqs() {
	std::vector<uint8_t> received;
	{
		std::lock_guard<std::mutex> lock(radioMutex);
		sx126x_irq_mask_t irq = 0;
		if (sx126x_get_and_clear_irq_status(&halContext, &irq) != SX126X_STATUS_OK) {
			return;
		}

		if (irq == 0) {
			return;
		}

		if (irq & SX126X_IRQ_RX_DONE) {
			sx126x_rx_buffer_status_t status{};
			if (sx126x_get_rx_buffer_status(&halContext, &status) == SX126X_STATUS_OK && status.pld_len_in_bytes > 0) {
				received.resize(status.pld_len_in_bytes);
				sx126x_read_buffer(&halContext, status.buffer_start_pointer, received.data(), status.pld_len_in_bytes);
			}
			restartRx();
		}

		if (irq & SX126X_IRQ_CRC_ERROR) {
			RCLCPP_WARN(this->get_logger(), "RX CRC error");
		}
	}

	if (!received.empty()) {
		handleInboundFrame(received);
	}
}


void RadioTransceiverNode::handleInboundFrame(const std::vector<uint8_t> &frame) {
	uint16_t port = 0;
	std::vector<uint8_t> payload;
	if (!decodeFrame(frame, port, payload)) {
		return;
	}

	lastInboundTime = this->now();

	if (port == rxMoveArmPort) {
		handleMoveArmCommand(payload);
	} else if (port == rxSetRadioParamsPort) {
		handleSetRadioParams(payload);
	} else if (port == rxRequestImagePort) {
		handleRequestImage(payload);
	} else {
		RCLCPP_DEBUG(this->get_logger(), "Unhandled inbound port %u", port);
	}
}

bool RadioTransceiverNode::decodeFrame(const std::vector<uint8_t> &frame, uint16_t &port,
								 		 std::vector<uint8_t> &payload) const {
	if (frame.size() < 2) {
		return false;
	}

	const uint16_t rxPreamble = static_cast<uint16_t>(frame[0]) |
									 static_cast<uint16_t>(frame[1] << 8);
	port = rxPreamble;

	payload.assign(frame.begin() + 2, frame.end());
    return true;
}

std::vector<uint8_t> RadioTransceiverNode::buildFrame(const std::vector<uint8_t> &payload) const {
	std::vector<uint8_t> frame;
	frame.reserve(payload.size() + 2);
	frame.push_back(static_cast<uint8_t>(txPort & 0xFF));
	frame.push_back(static_cast<uint8_t>((txPort >> 8) & 0xFF));
	frame.insert(frame.end(), payload.begin(), payload.end());
	return frame;
}

std::vector<uint8_t> RadioTransceiverNode::serializeHeartbeat() const {
	std::vector<uint8_t> out;
	out.reserve(21);
	writeUint32(out, lastHeartbeat.latitude);
	writeUint32(out, lastHeartbeat.longitude);
	writeUint16(out, lastHeartbeat.battery_voltage);
	writeUint16(out, lastHeartbeat.current_draw);
	writeUint32(out, lastHeartbeat.uptime);
	writeUint32(out, lastHeartbeat.gps_time);
	out.push_back(lastHeartbeat.status_bitfield);
	out.push_back(lastHeartbeat.sd_card_fill_percentage);
	out.push_back(lastHeartbeat.latest_picture_id);
	return out;
}

std::vector<uint8_t> RadioTransceiverNode::serializeArmTelemetry() const {
	std::vector<uint8_t> out;
	out.reserve(40);
	out.push_back(lastArmStatus.fault_status);
	writeFloat(out, lastArmStatus.m1_commanded_speed);
	writeFloat(out, lastArmStatus.m2_commanded_speed);
	writeFloat(out, lastArmStatus.m3_commanded_speed);
	writeFloat(out, lastArmStatus.servo_commanded_angle);
	writeFloat(out, lastArmStatus.shoulder_yaw);
	writeFloat(out, lastArmStatus.shoulder_pitch);
	writeFloat(out, lastArmStatus.elbow_angle);
	writeFloat(out, lastArmStatus.shoulder_yaw_limit_switch);
	writeFloat(out, lastArmStatus.temp1);
	writeFloat(out, lastArmStatus.temp2);
	writeUint16(out, lastArmCommand.command_number);
	return out;
}

void RadioTransceiverNode::handleMoveArmCommand(const std::vector<uint8_t> &payload) {
	if (payload.size() < 12) {
		RCLCPP_WARN(this->get_logger(), "MoveArm payload too short (%zu)", payload.size());
		return;
	}

	arm_msgs::msg::ArmCommand cmd;
	cmd.command_number = readUint16(payload, 0);
	cmd.shoulder_yaw = readInt16(payload, 2);
	cmd.shoulder_pitch = readInt16(payload, 4);
	cmd.elbow_angle = readInt16(payload, 6);
	cmd.wrist_angle = readInt16(payload, 8);
	cmd.take_picture = payload[10] != 0;
	cmd.transmit_from_new_position = payload[11] != 0;

	armCommandPub->publish(cmd);
	lastArmCommand = cmd;
	RCLCPP_INFO(this->get_logger(),
							"MoveArm from GS: cmd=%u yaw=%d pitch=%d elbow=%d wrist=%d take_pic=%d tx_new_pos=%d",
							cmd.command_number, cmd.shoulder_yaw, cmd.shoulder_pitch, cmd.elbow_angle, cmd.wrist_angle,
							cmd.take_picture, cmd.transmit_from_new_position);
}

void RadioTransceiverNode::handleSetRadioParams(const std::vector<uint8_t> &payload) {
	if (payload.size() < 19) {
		RCLCPP_WARN(this->get_logger(), "SetRadioParams payload too short (%zu)", payload.size());
		return;
	}

	RadioConfig newConfig = activeConfig;
	arm_msgs::msg::ArmCommand orient;
	orient.command_number = 0;
	orient.shoulder_yaw = readInt16(payload, 0);
	orient.shoulder_pitch = readInt16(payload, 2);
	orient.elbow_angle = readInt16(payload, 4);
	orient.wrist_angle = readInt16(payload, 6);
	orient.take_picture = false;
	orient.transmit_from_new_position = true;

	newConfig.spreadingFactor = std::clamp<uint8_t>(payload[8], 5, 12);
	newConfig.bandwidthHz = readUint32(payload, 9);
	newConfig.codingRate = readUint32(payload, 13);
	uint8_t rxTests = payload[17];
	uint8_t txTests = payload[18];

	applyRadioConfig(newConfig);
	armCommandPub->publish(orient);
	lastArmCommand = orient;
	scheduleRevertFromGsCommand(rxTests, txTests);
	RCLCPP_INFO(this->get_logger(),
							"Applied GS radio params: sf=%u bw=%u cr=%u; arm orient yaw=%d pitch=%d elbow=%d wrist=%d",
							newConfig.spreadingFactor, newConfig.bandwidthHz, newConfig.codingRate, orient.shoulder_yaw,
							orient.shoulder_pitch, orient.elbow_angle, orient.wrist_angle);
}

void RadioTransceiverNode::handleRequestImage(const std::vector<uint8_t> &payload) {
	if (payload.size() < 3) {
		RCLCPP_WARN(this->get_logger(), "RequestImage payload too short (%zu)", payload.size());
		return;
	}

	arm_msgs::msg::ImageRequest request;
	request.image_id = payload[0];
	request.compress = payload[1] != 0;
	request.quality = payload[2];

	RCLCPP_INFO(this->get_logger(), "Received image request from GS: ID=%u compress=%d quality=%u",
	            request.image_id, request.compress, request.quality);

	imageRequestPub->publish(request);
}

std::vector<uint8_t> RadioTransceiverNode::serializeImageData(
		const arm_msgs::msg::ImageData &imageData) const {
	std::vector<uint8_t> out;
	out.reserve(13 + imageData.data.size());
	
	out.push_back(imageData.image_id);
	writeUint32(out, imageData.total_chunks);
	writeUint32(out, imageData.chunk_index);
	writeUint32(out, imageData.total_size);
	out.insert(out.end(), imageData.data.begin(), imageData.data.end());
	
	return out;
}

sx126x_lora_bw_t RadioTransceiverNode::toLoraBandwidth(uint32_t bandwidth) const {
	if (bandwidth >= 500000) return SX126X_LORA_BW_500;
	if (bandwidth >= 250000) return SX126X_LORA_BW_250;
	if (bandwidth >= 125000) return SX126X_LORA_BW_125;
	if (bandwidth >= 62500) return SX126X_LORA_BW_062;
	if (bandwidth >= 41700) return SX126X_LORA_BW_041;
	if (bandwidth >= 31250) return SX126X_LORA_BW_031;
	if (bandwidth >= 20800) return SX126X_LORA_BW_020;
	if (bandwidth >= 15600) return SX126X_LORA_BW_015;
	if (bandwidth >= 10400) return SX126X_LORA_BW_010;
	return SX126X_LORA_BW_007;
}

sx126x_lora_cr_t RadioTransceiverNode::toLoraCodingRate(uint32_t codingRate) const {
	switch (codingRate) {
	case 5:
		return SX126X_LORA_CR_4_5;
	case 6:
		return SX126X_LORA_CR_4_6;
	case 7:
		return SX126X_LORA_CR_4_7;
	case 8:
	default:
		return SX126X_LORA_CR_4_8;
	}
}

uint8_t RadioTransceiverNode::computeLdro(uint8_t sf, uint32_t bandwidthHz) const {
	if (sf >= 11 && bandwidthHz <= 125000) {
		return 1;
	}
	if (sf == 10 && bandwidthHz <= 62500) {
		return 1;
	}
	return 0;
}

bool RadioTransceiverNode::configsEqual(const RadioConfig &a, const RadioConfig &b) const {
	return a.frequencyHz == b.frequencyHz && a.txPowerDbm == b.txPowerDbm &&
				 a.spreadingFactor == b.spreadingFactor && a.bandwidthHz == b.bandwidthHz &&
				 a.codingRate == b.codingRate && a.preambleLength == b.preambleLength;
}

void RadioTransceiverNode::scheduleRevertFromGsCommand(uint8_t rxTests, uint8_t txTests) {
	int extra = static_cast<int>(rxTests + txTests) * 2;
	int revertSeconds = std::max(radioRevertSec, extra);
	revertDeadline = this->now() + rclcpp::Duration::from_seconds(revertSeconds);
}

void RadioTransceiverNode::maybeRevertRadioConfig() {
	const auto now = this->now();
	if (revertDeadline && now > *revertDeadline) {
		if (!configsEqual(activeConfig, defaultConfig)) {
			applyRadioConfig(defaultConfig);
		}
		revertDeadline.reset();
	}

	if (gsTimeoutSec > 0 && lastInboundTime.nanoseconds() > 0) {
		if ((now - lastInboundTime).seconds() > gsTimeoutSec && !configsEqual(activeConfig, defaultConfig)) {
			RCLCPP_WARN(this->get_logger(), "No GS contact; reverting radio to defaults");
			applyRadioConfig(defaultConfig);
			revertDeadline.reset();
		}
	}
}

void RadioTransceiverNode::writeUint16(std::vector<uint8_t> &out, uint16_t value) const {
	out.push_back(static_cast<uint8_t>(value & 0xFF));
	out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

void RadioTransceiverNode::writeUint32(std::vector<uint8_t> &out, uint32_t value) const {
	out.push_back(static_cast<uint8_t>(value & 0xFF));
	out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
	out.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
	out.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
}

void RadioTransceiverNode::writeFloat(std::vector<uint8_t> &out, float value) const {
	static_assert(sizeof(float) == 4, "float is not 32-bit");
	uint32_t raw;
	std::memcpy(&raw, &value, sizeof(float));
	writeUint32(out, raw);
}

uint16_t RadioTransceiverNode::readUint16(const std::vector<uint8_t> &in, size_t offset) const {
	if (offset + 1 >= in.size()) {
		return 0;
	}
	return static_cast<uint16_t>(in[offset]) | (static_cast<uint16_t>(in[offset + 1]) << 8);
}

int16_t RadioTransceiverNode::readInt16(const std::vector<uint8_t> &in, size_t offset) const {
	return static_cast<int16_t>(readUint16(in, offset));
}

uint32_t RadioTransceiverNode::readUint32(const std::vector<uint8_t> &in, size_t offset) const {
	if (offset + 3 >= in.size()) {
		return 0;
	}
	return static_cast<uint32_t>(in[offset]) |
				 (static_cast<uint32_t>(in[offset + 1]) << 8) |
				 (static_cast<uint32_t>(in[offset + 2]) << 16) |
				 (static_cast<uint32_t>(in[offset + 3]) << 24);
}
