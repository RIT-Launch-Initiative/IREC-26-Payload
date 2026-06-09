#include "cubesat_captain/expert.hpp"
namespace cubesat_captain {

struct ArmPose {
    int8_t shoulder_yaw;
    int8_t shoulder_pitch;
    int8_t elbow_pitch;
    int8_t wrist_pitch;

    bool isCloseEnoughTo(const ArmPose &other) const;
};

enum class ArmState {
    Unfolding,
    Panoramaing,
};

class ArmExpert : public Expert {
  public:
    using ExtendArm = cubesat_msgs::action::ExtendArm;
    using GoalHandleExtendArm = rclcpp_action::ClientGoalHandle<ExtendArm>;
    using FlipServoAction = cubesat_msgs::action::FlipServoAction;
    using GoalHandleFlipServoAction = rclcpp_action::ClientGoalHandle<FlipServoAction>;

    ArmExpert(rclcpp::Logger logger, Levers &levers, ArmState state) : Expert(logger, levers), for_state(state) {}
    ~ArmExpert() {}

    void enter_state() override;


    void arm_response_cb(GoalHandleExtendArm::SharedPtr);
    void arm_result_cb(const GoalHandleExtendArm::WrappedResult &);
    void arm_feedback_cb(GoalHandleExtendArm::SharedPtr, const std::shared_ptr<const ExtendArm::Feedback> feedback);

  private:
    void send_target(const ArmPose &pose, bool ignore_stall);
    void decide_next();
    void finish_good();
    ArmState for_state;
    size_t path_index = 0;
    size_t attempts_for_this_side = 0;
};
} // namespace cubesat_captain