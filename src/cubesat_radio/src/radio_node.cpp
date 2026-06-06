#include "cubesat_radio/radio_node.hpp"

#include "rclcpp/serialization.hpp"
#include <chrono>
#include <fstream>
#include <utility>

namespace cubesat_radio {

RadioNode::RadioNode(const rclcpp::NodeOptions &options) : rclcpp::Node("radio_node", options) {
    const auto hardware = loadHardwareConfig();
    RadioProfile profile = loadParameterProfile();
    // if (!loadProfileFromFile(flight_dir)) {
    // serializeProfile(flight_dir + "/radio_profile");
    // }

    statePub = create_publisher<cubesat_msgs::msg::RadioState>("radio/state", 10);

    rxPacketPub = create_publisher<cubesat_msgs::msg::RadioPacket>("radio/rx_packet", 10);

    txPacketSub = create_subscription<cubesat_msgs::msg::RadioPacket>(
        "radio/tx_packet", 68, std::bind(&RadioNode::handleTxPacket, this, std::placeholders::_1));

    sendRadioPacketSrv = create_service<cubesat_msgs::srv::SendRadioPacket>(
        "radio/send_packet",
        std::bind(&RadioNode::handleSendRadioPacket, this, std::placeholders::_1, std::placeholders::_2));

    radio = std::make_unique<Sx1262Radio>(hardware);

    if (!radio->open()) {
        RCLCPP_WARN(get_logger(), "Radio hardware open failed; verify SPI device, "
                                  "GPIO lines, and SX1262 wiring");
        return;
    }

    if (!radio->configure(profile)) {
        RCLCPP_WARN(get_logger(), "Radio configure failed; verify LoRa profile and "
                                  "SX1262 initialization sequence");
        return;
    }

    if (!radio->setReceiveMode()) {
        RCLCPP_WARN(get_logger(), "Radio RX mode enable failed after configuration");
        return;
    }

    RCLCPP_INFO(get_logger(),
                "Radio initialized: freq=%u Hz bw=%u Hz sf=%u cr=4/%u tx=%d dBm "
                "spi=%s reset=%d busy=%d dio1=%d",
                profile.frequency_hz, profile.bandwidth_hz, profile.spreading_factor, profile.coding_rate,
                profile.tx_power_dbm, hardware.spi_device.c_str(), hardware.reset_gpio, hardware.busy_gpio,
                hardware.dio1_gpio);

    rsm.radio_flag_signal.store(0);
    rxThread = std::thread(&RadioNode::radioLoop, this);

    rsm.interrupt_thread_running.store(true);
    interruptThread = std::thread(&RadioNode::interruptLoop, this);
}

RadioNode::~RadioNode() {
    RCLCPP_INFO(get_logger(), "Stopping interrupt and radio threads");

    rsm.signalStopping();

    RCLCPP_INFO(get_logger(), "joining interrupt thread");
    if (interruptThread.joinable()) {
        interruptThread.join();
    }

    RCLCPP_INFO(get_logger(), "joining radio thread");
    if (rxThread.joinable()) {
        rxThread.join();
    }
}

RadioProfile RadioNode::loadParameterProfile() {
    RadioProfile profile;
    profile.frequency_hz = static_cast<uint32_t>(declare_parameter<int64_t>("frequency_hz", profile.frequency_hz));
    profile.bandwidth_hz = static_cast<uint32_t>(declare_parameter<int64_t>("bandwidth_hz", profile.bandwidth_hz));
    profile.spreading_factor =
        static_cast<uint8_t>(declare_parameter<int64_t>("spreading_factor", profile.spreading_factor));
    profile.coding_rate = static_cast<uint8_t>(declare_parameter<int64_t>("coding_rate", profile.coding_rate));
    profile.tx_power_dbm = static_cast<int8_t>(declare_parameter<int64_t>("tx_power_dbm", profile.tx_power_dbm));
    return profile;
}

RadioHardwareConfig RadioNode::loadHardwareConfig() {
    RadioHardwareConfig hardware;
    hardware.spi_device = declare_parameter<std::string>("spi_device", hardware.spi_device);
    hardware.spi_speed_hz = static_cast<uint32_t>(declare_parameter<int64_t>("spi_speed_hz", hardware.spi_speed_hz));
    hardware.gpio_chip_name = declare_parameter<std::string>("gpio_chip_name", hardware.gpio_chip_name);
    hardware.reset_gpio = static_cast<int>(declare_parameter<int64_t>("reset_gpio", hardware.reset_gpio));
    hardware.busy_gpio = static_cast<int>(declare_parameter<int64_t>("busy_gpio", hardware.busy_gpio));
    hardware.dio1_gpio = static_cast<int>(declare_parameter<int64_t>("dio1_gpio", hardware.dio1_gpio));
    hardware.tx_enable_gpio = static_cast<int>(declare_parameter<int64_t>("tx_enable_gpio", hardware.tx_enable_gpio));
    hardware.rx_enable_gpio = static_cast<int>(declare_parameter<int64_t>("rx_enable_gpio", hardware.rx_enable_gpio));
    hardware.tx_enable_active_high = declare_parameter<bool>("tx_enable_active_high", hardware.tx_enable_active_high);
    hardware.rx_enable_active_high = declare_parameter<bool>("rx_enable_active_high", hardware.rx_enable_active_high);
    return hardware;
}

void RadioNode::interruptLoop() {
    while (rclcpp::ok() && rsm.interrupt_thread_running.load()) {
        if (!radio->waitForInterrupt(std::chrono::milliseconds(200))) {
            continue;
        }
        rsm.signalInterrupt();
    }
}

void RadioNode::RadioStateMachine::signalInterrupt() {
    radio_flag_signal.fetch_or(INTERRUPT_BIT);
    radio_flag_signal.notify_one();
}
void RadioNode::RadioStateMachine::signalStopping() {
    radio_flag_signal.fetch_or(STOPPING_BIT);
    radio_flag_signal.notify_one();

    interrupt_thread_running.store(false);
    interrupt_thread_running.notify_one();
}

bool RadioNode::RadioStateMachine::submitPacketToSend(std::vector<uint8_t> packet) {
    RCLCPP_INFO(get_logger(), "Submitting packet of length %ld to queue", packet.size());
    {
        std::lock_guard lk{queue_and_profile_mutex};
        outbound_queue.push(packet);
        radio_flag_signal.fetch_or(NEW_PACKET_BIT);
        radio_flag_signal.notify_one();
    }
    return true;
}
void RadioNode::RadioStateMachine::setProfileNow(RadioProfile prof) {
    std::lock_guard lk{queue_and_profile_mutex};
    incomingProfile = prof;
    radio_flag_signal.fetch_or(CONFIG_CHANGED_BIT);
    radio_flag_signal.notify_one();
}

rclcpp::Logger RadioNode::RadioStateMachine::get_logger() { return rclcpp::get_logger("rsm"); }

void RadioNode::radioLoop() {
    std::queue<std::vector<uint8_t>> about_to_send{};
    bool stillHaveWork = false;

    while (rclcpp::ok()) {
        // if still have work, iterate so we catch new events but don't block
        if (!stillHaveWork) {
            RCLCPP_DEBUG(get_logger(), "Waiting for work");
            rsm.radio_flag_signal.wait(0); // handled everything last iter, wait for anything
        }

        uint32_t status = rsm.radio_flag_signal.load();
        rsm.radio_flag_signal.store(0);

        bool packet_ready = (status & rsm.NEW_PACKET_BIT) != 0;
        bool interrupt_happened = (status & rsm.INTERRUPT_BIT) != 0;
        bool stop_happened = (status & rsm.STOPPING_BIT) != 0;
        bool immediate_reconfig_happened = (status & rsm.CONFIG_CHANGED_BIT) != 0;
        bool rx_chance_expired = (status & rsm.RX_CHANCE_EXPIRED_BIT) != 0;
        bool link_test_chance_expired = (status & rsm.LINK_TEST_CHANCE_EXPIRED_BIT) != 0;

        bool rx_happened = false;
        bool link_configure_happened = false;
        bool link_test_happened = false;
        bool expect_more_rx = false;

        if (stop_happened) {
            break;
        }

        // take packets from the external queue, add them to our queue, transmit them later
        if (packet_ready) {
            std::lock_guard lk{rsm.queue_and_profile_mutex};
            while (!rsm.outbound_queue.empty()) {
                about_to_send.push(rsm.outbound_queue.front());
                rsm.outbound_queue.pop();
                RCLCPP_INFO(get_logger(), "Got Packet");
            }
        }

        if (interrupt_happened) {
            // maybe check if interrupt happened bc of rx or some other reason
            auto maybe_packet = radio->receive();
            if (maybe_packet) {
                auto packet = *maybe_packet;
                cubesat_msgs::msg::RadioPacket msg;
                msg.stamp = now();
                msg.data = packet.data;
                msg.rssi = packet.rssi_dbm;
                msg.snr = packet.snr_db;
                rxPacketPub->publish(std::move(msg));
                rx_happened = true;
            }
            radio->dumpStatus();
            // reset rx timer
        }
        if (immediate_reconfig_happened) {
            {
                std::lock_guard lk{rsm.queue_and_profile_mutex};
                if (radio->configure(rsm.incomingProfile)) {
                    rsm.stableProfile = rsm.incomingProfile;
                    RCLCPP_INFO(get_logger(), "Immediate reconfigure of radio succeeded");
                } else {
                    RCLCPP_INFO(get_logger(), "Immediate reconfigure of radio failed");
                }
            }
        }
        // now that we've handled the incoming info, we need to decide what to do
        if (link_test_chance_expired) {
            rsm.processEvent(*this, RSM::EventType::LinkTestTimeExpired);
        }
        if (rx_happened) {
            if (link_configure_happened) {
                // set profile under test
                RCLCPP_INFO(get_logger(), "Link Configure happened");
                rsm.processEvent(*this, RSM::EventType::LinkConfigHeard);
            }
            if (link_test_happened) {
                RCLCPP_INFO(get_logger(), "Heard Link Test");
                rsm.processEvent(*this, RSM::EventType::LinkTestHeard);
            }
            if (expect_more_rx) {
                RCLCPP_INFO(get_logger(), "Received packet and expecting more");
                rsm.processEvent(*this, RSM::EventType::RxAndContinue);
            } else {
                RCLCPP_INFO(get_logger(), "Received packet and NOT expecting more");
                rsm.processEvent(*this, RSM::EventType::RxSingle);
            }
        }
        if (rx_chance_expired) {
            RCLCPP_INFO(get_logger(), "Receiving time expired");
            rsm.processEvent(*this, RSM::EventType::RxTimeExpired);
        }

        if (!rsm.isLinkTesting) {
            RCLCPP_WARN(get_logger() ,"Num transmitted in a row %d queue size %ld", rsm.numTransmittedInARow, about_to_send.size());
            if (rsm.state == RSM::NormalState::Idle) {
                if (about_to_send.size() > 0) {
                    radio->send(about_to_send.front());
                    about_to_send.pop();
                }
                rsm.numTransmittedInARow++;
                if (about_to_send.size() > 0) {
                    stillHaveWork = true;
                } else {
                    stillHaveWork = false;
                }
                radio->dumpStatus();

            } else {
                // if rxing, just rx
                stillHaveWork = false;
            }
        }
    }
}
void RadioNode::RadioStateMachine::processEvent(RadioNode &node, EventType event) {
    switch (event) {
    case EventType::RxAndContinue:
        state = NormalState::RxEnforced;
        break;
    case EventType::RxSingle:
        state = NormalState::Idle;
        break;
    case EventType::RxTimeExpired:
        state = NormalState::Idle;
        break;
    case EventType::LinkConfigHeard: {
        RCLCPP_INFO(get_logger(),
                    "Link Configure happened. Transmitting Link Configure acknowledge and switching parameters");
        // TODO!!!!
        // radio->send()
        // radio->configure(profileUnderTest);
        // start link test timer
    } break;
    case EventType::LinkTestTimeExpired:
        isLinkTesting = false;
        node.radio->configure(stableProfile);
        // TODO send nack
        break;
    case EventType::LinkTestHeard:
        // send link test ack
        stableProfile = profileUnderTest;
        isLinkTesting = false;
        break;
    }
}
void RadioNode::handleTxPacket(const cubesat_msgs::msg::RadioPacket::SharedPtr msg) {
    if (msg == nullptr) {
        return;
    }

    if (rsm.submitPacketToSend(msg->data)) {
        RCLCPP_INFO(get_logger(), "Queued radio TX from topic: %zu bytes", msg->data.size());
    }
}

void RadioNode::handleSendRadioPacket(const std::shared_ptr<cubesat_msgs::srv::SendRadioPacket::Request> &request,
                                      std::shared_ptr<cubesat_msgs::srv::SendRadioPacket::Response> response) {
    response->success = request != nullptr && rsm.submitPacketToSend(request->data);
}

// bool RadioNode::loadProfileFromFile(std::string path, RadioProfile &into) {
//     try {
//         // read file
//         std::ifstream file(path, std::ios::in | std::ios::binary | std::ios::ate);
//         file.exceptions(std::ofstream::failbit | std::ofstream::badbit);

//         std::streamsize size = file.tellg();
//         file.seekg(0, std::ios::beg);

//         std::vector<uint8_t> buffer(size);
//         file.read(reinterpret_cast<char *>(buffer.data()), size);
//         file.close();

//         rclcpp::SerializedMessage serialized_msg;
//         auto &rcl_msg = serialized_msg.get_rcl_serialized_message();

//         // deserialize
//         rmw_serialized_message_resize(&rcl_msg, size);
//         std::memcpy(rcl_msg.buffer, buffer.data(), size);
//         rcl_msg.buffer_length = size; // Explicitly set length to avoid invalid state errors

//         rclcpp::Serialization<cubesat_msgs::msg::LoraParameters> serializer;
//         serializer.deserialize_message(&serialized_msg, &profile);
//         return true;
//     } catch (std::exception &e) {
//         RCLCPP_WARN(get_logger(), "Failed to load lora profile from file: %s", e.what());
//         return false;
//     }
// }

// void RadioNode::serializeProfile(std::string path, RadioProfile profile) {

//     rclcpp::Serialization<cubesat_msgs::msg::LoraParameters> serializer;
//     rclcpp::SerializedMessage serialized_msg;
//     serializer.serialize_message(&profile, &serialized_msg);

//     std::ofstream file(path, std::ios::out | std::ios::binary);
//     file.write(reinterpret_cast<const char *>(serialized_msg.get_rcl_serialized_message().buffer),
//                serialized_msg.get_rcl_serialized_message().buffer_length);
//     file.close();
// }

} // namespace cubesat_radio
