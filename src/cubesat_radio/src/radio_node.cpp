#include "cubesat_radio/radio_node.hpp"

#include "rclcpp/serialization.hpp"
#include <chrono>
#include <fstream>
#include <utility>

#include "cubesat_comms/lora.h"
#include "cubesat_comms/packets_g2p.h"
#include "cubesat_comms/packets_p2g.h"

namespace cubesat_radio {
SpreadingFactor psf_to_lsf(uint8_t sf) {
    switch (sf) {
    case 7:
        return SF_7;
    case 8:
        return SF_8;
    case 9:
        return SF_9;
    case 10:
        return SF_10;
    case 11:
        return SF_11;
    case 12:
        return SF_12;
    }
    return SF_12;
}

Bandwidth pbw_to_lbw(uint32_t bandwidth_hz) {
    if (bandwidth_hz >= 500000) {
        return BW_500;
    }
    if (bandwidth_hz >= 250000) {
        return BW_250;
    }
    if (bandwidth_hz >= 125000) {
        return BW_125;
    }
    if (bandwidth_hz >= 62500) {
        return BW_62_5;
    }
    return BW_125;
}
CodingRate pcr_to_lcr(uint8_t coding_rate) {
    switch (coding_rate) {
    case 5:
        return CodingRate::CR_4_5;
    case 6:
        return CodingRate::CR_4_6;
    case 7:
        return CodingRate::CR_4_7;
    case 8:
        return CodingRate::CR_4_8;
    default:
        return CodingRate::CR_4_5;
    }
}

uint8_t lsf_to_psf(SpreadingFactor sf) {
    switch (sf) {
    case SF_7:
        return 7;
    case SF_8:
        return 8;
    case SF_9:
        return 9;
    case SF_10:
        return 10;
    case SF_11:
        return 11;
    case SF_12:
        return 12;
    default:
        return SF_12;
    }
}

uint32_t lbw_to_pbw(Bandwidth bw) {
    switch (bw) {
    case Bandwidth::BW_62_5:
        return 62500;
    case Bandwidth::BW_125:
        return 125000;
    case Bandwidth::BW_250:
        return 250000;
    case Bandwidth::BW_500:
        return 500000;
    }
    return 125000;
}
uint8_t lcr_to_pcr(CodingRate coding_rate) {
    switch (coding_rate) {
    case CodingRate::CR_4_5:
        return 5;
    case CodingRate::CR_4_6:
        return 6;
    case CodingRate::CR_4_7:
        return 7;
    case CodingRate::CR_4_8:
        return 8;
    default:
        return 8;
    }
}

std::vector<uint8_t> link_change_acknowledge_packet(const RadioProfile &profile) {
    SpreadingFactor sf = psf_to_lsf(profile.spreading_factor);
    Bandwidth bw = pbw_to_lbw(profile.bandwidth_hz);
    CodingRate cr = pcr_to_lcr(profile.coding_rate);

    LoraLinkChange change{
        .num_test_packets = 1,
        .dbm = profile.tx_power_dbm,
        .freq = profile.frequency_hz,
        .sf = sf,
        .bw = bw,
        .cr = cr,
    };
    std::vector<uint8_t> packet{0};
    packet.resize(SIZEOF_PACKED_LORA_LINK_CHANGE + 1);
    P2GLinkHeader head{P2GPacketType_LinkControl, 0};
    pack_p2g_link_header(&head, packet.data());
    pack_lora_link_change(&change, &packet.at(1));
    return packet;
}

std::vector<uint8_t> link_test_acknowledge() {
    std::vector<uint8_t> packet{0};
    packet.resize(1 + 26);

    P2GLinkHeader head{P2GPacketType_LinkTestResponse, 0};
    pack_p2g_link_header(&head, packet.data());
    for (size_t i = 0; i < 26; i++) {
        packet[i + 1] = 'a' + i;
    }
    return packet;
}

void put_queue_length_to_header(std::vector<uint8_t> &packet, size_t queue_length) {
    if (queue_length > MAX_PACKETS_BEFORE_RESPONSE) {
        queue_length = MAX_PACKETS_BEFORE_RESPONSE;
    }
    packet[0] &= 0b11000000;
    packet[0] |= (uint8_t)queue_length;
}
std::optional<RadioProfile> change_packet_to_profile(const std::vector<uint8_t> &packet) {
    LoraLinkChange change;
    if (unpack_lora_link_change(packet.data() + 1, packet.size() - 1, &change) == UnpackResult_AllGood) {
        RadioProfile prof{
            .frequency_hz = change.freq,
            .bandwidth_hz = lbw_to_pbw(change.bw),
            .spreading_factor = lsf_to_psf(change.sf),
            .coding_rate = lcr_to_pcr(change.cr),
            .tx_power_dbm = change.dbm,
        };
        return prof;
    }
    return std::nullopt;
}

std::optional<G2PLinkHeader> header_of_packet(const std::vector<uint8_t> &pack) {
    G2PLinkHeader header;
    if (unpack_g2p_link_header(pack.data(), pack.size(), &header) == UnpackResult_AllGood) {
        return header;
    }
    return std::nullopt;
}

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
    rsm.stableProfile = profile;

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

    wait_for_link_test_timer =
        create_wall_timer(std::chrono::milliseconds(30 * 1000), [this] { this->rsm.linkTestChanceExpired(); });
    wait_for_link_test_timer->cancel();

    watchdog_timer = create_wall_timer(std::chrono::seconds(5 * 60), [this] {
        RCLCPP_WARN(get_logger(), "Havent received anything in a while. Shutting down");
        rclcpp::shutdown();
    });

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

// void RadioNode::onEnforcedRxTimer() {
//     enforced_rx_timer->cancel();
//     rsm.rxChanceExpired();
//     RCLCPP_INFO(get_logger(), "enforced rx time is up");
// }

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

void RadioNode::RadioStateMachine::linkTestChanceExpired() {
    radio_flag_signal.fetch_or(LINK_TEST_CHANCE_EXPIRED_BIT);
    radio_flag_signal.notify_one();
}

bool RadioNode::RadioStateMachine::submitPacketToSend(std::vector<uint8_t> packet) {
    if (packet.empty() || packet.size() > 255) {
        RCLCPP_WARN(get_logger(), "Dropping packet of bad length %ld", packet.size());
        return false;
    }
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
    size_t iter = 0;

    while (rclcpp::ok()) {
        iter++;
        // if still have work, iterate so we catch new events but don't block
        if (!stillHaveWork) {
            RCLCPP_DEBUG(get_logger(), "Waiting for work");
            rsm.radio_flag_signal.wait(0); // handled everything last iter, wait for anything
        }

        // exchange so a flag set between a load and a separate store can't be lost
        uint32_t status = rsm.radio_flag_signal.exchange(0);

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
                RCLCPP_DEBUG(get_logger(), "Packet Queued");
            }
        }

        if (interrupt_happened) {
            auto maybe_packet = radio->receive();
            if (maybe_packet) {
                watchdog_timer->reset();
                auto packet = *maybe_packet;
                cubesat_msgs::msg::RadioPacket msg;
                msg.stamp = now();
                msg.data = packet.data;
                msg.rssi = packet.rssi_dbm;
                msg.snr = packet.snr_db;
                rxPacketPub->publish(std::move(msg));
                RCLCPP_INFO(get_logger(), "Received packet of size: %ld processing....", msg.data.size());
                rx_happened = true;

                auto maybe_header = header_of_packet(packet.data);
                if (maybe_header) {
                    G2PLinkHeader header = *maybe_header;
                    if (header.packet_type == G2PPacketType_LinkControl) {
                        auto newProf = change_packet_to_profile(packet.data);
                        if (newProf) {
                            rsm.profileUnderTest = *newProf;
                            link_configure_happened = true;
                        }

                    } else if (header.packet_type == G2PPacketType_LinkTest) {
                        link_test_happened = true;
                    }
                    if (header.expected_packets_before_response > 0) {
                        expect_more_rx = true;
                    }
                }
            }
            radio->dumpStatus();
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
                radio->dumpStatus();

                if (!radio->setReceiveMode()) {
                    // don't return: that would kill the radio thread on a transient failure
                    RCLCPP_WARN(get_logger(), "Radio RX mode enable failed after link change");
                }
                radio->dumpStatus();
            }
        }
        // now that we've handled the incoming info, we need to decide what to do
        if (link_test_chance_expired) {
            rsm.processEvent(*this, RSM::EventType::LinkTestTimeExpired, about_to_send.size());
        }
        if (rx_happened) {
            if (link_configure_happened) {
                // set profile under test
                RCLCPP_INFO(get_logger(), "Link Configure happened");
                rsm.processEvent(*this, RSM::EventType::LinkConfigHeard, about_to_send.size());
            }
            if (link_test_happened) {
                RCLCPP_INFO(get_logger(), "Heard Link Test");
                rsm.processEvent(*this, RSM::EventType::LinkTestHeard, about_to_send.size());
            }
            if (expect_more_rx) {
                RCLCPP_INFO(get_logger(), "Received packet and expecting more");
                rsm.processEvent(*this, RSM::EventType::RxAndContinue, about_to_send.size());
            } else {
                RCLCPP_INFO(get_logger(), "Received packet and NOT expecting more");
                rsm.processEvent(*this, RSM::EventType::RxSingle, about_to_send.size());
            }
        }
        if (rx_chance_expired) {
            RCLCPP_INFO(get_logger(), "Receiving time expired");
            rsm.processEvent(*this, RSM::EventType::RxTimeExpired, about_to_send.size());
        }

        if (!rsm.isLinkTesting) {
            if (rsm.state == RSM::NormalState::Idle) {
                if (about_to_send.size() > 0) {
                    RCLCPP_WARN(get_logger(), "TXing:  queue size %ld", about_to_send.size());

                    std::vector<uint8_t> packet = about_to_send.front();
                    about_to_send.pop();
                    put_queue_length_to_header(packet, about_to_send.size());
                    radio->send(packet);
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
                // rsm.numTransmittedInARow = 0;
                // enforced_rx_timer.reset();
            }
        } else {
            RCLCPP_INFO(get_logger(), "Skipping send because isLinkTesting");
        }
    }
}

// RadioNode::RadioStateMachine::RadioStateMachine(RadioNode &node) : parent(node) {}

void RadioNode::RadioStateMachine::processEvent(RadioNode &node, EventType event, size_t queue_length) {
    switch (event) {
    case EventType::RxAndContinue:
        state = NormalState::RxEnforced;
        // parent.enforced_rx_timer->reset();
        break;
    case EventType::RxSingle:
        state = NormalState::Idle;
        break;
    case EventType::RxTimeExpired:
        // RCLCPP_INFO(get_logger(), "Enforced RX Time expired. Going back to idle");
        state = NormalState::Idle;
        break;
    case EventType::LinkConfigHeard: {
        RCLCPP_INFO(get_logger(),
                    "Link Configure happened. Transmitting Link Configure acknowledge and switching parameters");
        auto packet = link_change_acknowledge_packet(profileUnderTest);
        if (!node.radio->send(packet)) {
            RCLCPP_WARN(get_logger(), "Failed to send link change acknowledge");
        }
        if (!node.radio->configure(profileUnderTest)) {
            RCLCPP_WARN(get_logger(), "Failed to configure for link change");
        }
        node.radio->dumpStatus();
        if (!node.radio->setReceiveMode()) {
            RCLCPP_WARN(get_logger(), "Failed to set recv mode after configure link change");
        }
        node.radio->dumpStatus();

        isLinkTesting = true;
        node.wait_for_link_test_timer->reset();
    } break;
    case EventType::LinkTestTimeExpired:
        profileUnderTest = stableProfile;
        RCLCPP_INFO(get_logger(), "Link test time expired, going back");
        isLinkTesting = false;
        if (!node.radio->configure(stableProfile)) {
            RCLCPP_WARN(get_logger(), "Failed to configure during revert to old params");
        }
        node.radio->dumpStatus();

        node.wait_for_link_test_timer->cancel();
        if (!node.radio->setReceiveMode()) {
            RCLCPP_WARN(get_logger(), "Radio RX mode enable failed after link change revert");
            return;
        }
        node.radio->dumpStatus();
        break;
    case EventType::LinkTestHeard:
        RCLCPP_INFO(get_logger(), "Link test heard, sticking with it");
        // send link test ack
        std::vector<uint8_t> packet = link_test_acknowledge();
        put_queue_length_to_header(packet, queue_length);

        if (!node.radio->send(packet)) {
            RCLCPP_WARN(get_logger(), "Failed to send link test acknowledge");
        }
        node.radio->dumpStatus();
        stableProfile = profileUnderTest;
        isLinkTesting = false;
        node.wait_for_link_test_timer->cancel();
        break;
    }
}
void RadioNode::handleTxPacket(const cubesat_msgs::msg::RadioPacket::SharedPtr msg) {
    if (msg == nullptr) {
        return;
    }

    if (rsm.submitPacketToSend(msg->data)) {
        RCLCPP_DEBUG(get_logger(), "Queued radio TX from topic: %zu bytes", msg->data.size());
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
