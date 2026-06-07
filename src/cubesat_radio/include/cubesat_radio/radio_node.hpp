#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "cubesat_msgs/msg/lora_parameters.hpp"
#include "cubesat_msgs/msg/radio_packet.hpp"
#include "cubesat_msgs/msg/radio_state.hpp"
#include "cubesat_msgs/srv/send_radio_packet.hpp"

#include "cubesat_radio/radio_profile.hpp"
#include "cubesat_radio/radio_types.hpp"
#include "cubesat_radio/sx1262_radio.hpp"

namespace cubesat_radio {

class RadioNode : public rclcpp::Node {
  public:
    explicit RadioNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~RadioNode() override;

  private:
    RadioProfile loadParameterProfile();
    /**
     * Modifies member
     * If no parameter file was found, dont overwrite anything
     * return true if read
     */
    // bool loadProfileFromFile(std::string path);
    void serializeProfile(std::string path);

    RadioHardwareConfig loadHardwareConfig();

    void radioLoop();
    void interruptLoop();

    // void onEnforcedRxTimer();

    // ros handlers
    void handleTxPacket(const cubesat_msgs::msg::RadioPacket::SharedPtr msg);
    void handleSendRadioPacket(const std::shared_ptr<cubesat_msgs::srv::SendRadioPacket::Request> &request,
                               std::shared_ptr<cubesat_msgs::srv::SendRadioPacket::Response> response);

    std::string flight_dir;

    rclcpp::Publisher<cubesat_msgs::msg::RadioState>::SharedPtr statePub;
    rclcpp::Publisher<cubesat_msgs::msg::RadioPacket>::SharedPtr rxPacketPub;

    rclcpp::Subscription<cubesat_msgs::msg::RadioPacket>::SharedPtr txPacketSub;
    rclcpp::Service<cubesat_msgs::srv::SendRadioPacket>::SharedPtr sendRadioPacketSrv;

    std::thread interruptThread;
    std::thread rxThread;
    std::unique_ptr<Sx1262Radio> radio;

    struct RadioStateMachine {
        // RadioStateMachine(RadioNode &node);
        // STATE
        enum NormalState {
            Idle,
            RxEnforced,
        };
        enum EventType {
            RxAndContinue,       // received a packet and it said more were coming
            RxSingle,            // received a packet and it said it was the last
            RxTimeExpired,       // silent period while waiting for the GS to say something ran out
            LinkConfigHeard,     // just acknowledeged a link change, time to listen for a link test
            LinkTestTimeExpired, // silent period waiting for the GS to say can you hear me expired
            LinkTestHeard,
        };

        // INTERACTION (safe to call from other places)
        void signalStopping();
        void signalInterrupt();
        bool submitPacketToSend(std::vector<uint8_t> packet);
        void setProfileNow(RadioProfile prof);

        void rxChanceExpired();
        void linkTestChanceExpired();

        rclcpp::Logger get_logger();

        static constexpr uint32_t STOPPING_BIT = 1;
        static constexpr uint32_t INTERRUPT_BIT = 2;
        static constexpr uint32_t NEW_PACKET_BIT = 4;
        static constexpr uint32_t CONFIG_CHANGED_BIT = 8;
        static constexpr uint32_t RX_CHANCE_EXPIRED_BIT = 16;
        static constexpr uint32_t LINK_TEST_CHANCE_EXPIRED_BIT = 32;

        // Communication state
        // RadioNode &parent;

        std::atomic<uint32_t> radio_flag_signal{0}; // cfg change, new packet, interrupt, stopping
        std::atomic<bool> interrupt_thread_running{true};

        std::mutex queue_and_profile_mutex;
        std::queue<std::vector<uint8_t>> outbound_queue;
        RadioProfile incomingProfile;

        // STATE
        void processEvent(RadioNode &node, EventType ev);
        NormalState state{NormalState::Idle};
        RadioProfile stableProfile;
        uint32_t numTransmittedInARow = 0;

        bool isLinkTesting = false;
        uint8_t linkTestNumLeft = 0;
        RadioProfile profileUnderTest;
    };
    // friend RadioStateMachine;

    using RSM = RadioStateMachine;
    // ONLY TOUCHED BY RADIO THREAD
    RadioStateMachine rsm;//{*this};
    // rclcpp::TimerBase::SharedPtr enforced_rx_timer;
};
} // namespace cubesat_radio
