#ifndef STEPIT_ROBOT_UNITREE2_QUADRUPED_H_
#define STEPIT_ROBOT_UNITREE2_QUADRUPED_H_

#include <array>
#include <memory>
#include <mutex>
#include <set>

#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <stepit/robot.h>
#include <stepit/robot/unitree2/common.h>

namespace u2_msg = unitree_go::msg::dds_;

namespace stepit {
template <typename Spec>
class UnitreeQuadrupedApi : public RobotApi {
 public:
  UnitreeQuadrupedApi() : RobotApi(Spec::robotName()), low_state_(getDoF(), getNumLegs()) {
    low_cmd_.head()[0]    = 0xFE;
    low_cmd_.head()[1]    = 0xEF;
    low_cmd_.level_flag() = 0xFF;
    low_cmd_.gpio()       = 0;
    for (auto &motor_cmd : low_cmd_.motor_cmd()) {
      motor_cmd.mode() = Spec::kMotorServoMode;
      motor_cmd.q()    = kPosStop;
      motor_cmd.dq()   = kVelStop;
      motor_cmd.kp()   = 0;
      motor_cmd.kd()   = 0;
      motor_cmd.tau()  = 0;
    }
    std::set<std::size_t> mapped_indices;
    for (std::size_t i{}; i < getDoF(); ++i) {
      const auto motor_index = Spec::motorIndex(i);
      STEPIT_ASSERT(motor_index < low_cmd_.motor_cmd().size(),
                    "Robot '{}' maps joint {} to out-of-range DDS motor index {}.",
                    Spec::robotName(), i, motor_index);
      STEPIT_ASSERT(mapped_indices.insert(motor_index).second,
                    "Robot '{}' maps more than one joint to DDS motor index {}.",
                    Spec::robotName(), motor_index);
      const float direction = Spec::motorDirection(i);
      STEPIT_ASSERT(direction == -1.0F or direction == 1.0F,
                    "Robot '{}' motor direction at joint {} must be -1 or 1.",
                    Spec::robotName(), i);
    }
  }

  void getControl(bool enable) override {
    if (not enable) return;

    Unitree2Dds::initialize();
    Unitree2MotionSwitcher::deactivate();

    low_cmd_pub_   = std::make_shared<u2_sdk::ChannelPublisher<u2_msg::LowCmd_>>("rt/lowcmd");
    low_state_sub_ = std::make_shared<u2_sdk::ChannelSubscriber<u2_msg::LowState_>>("rt/lowstate");
    low_cmd_pub_->InitChannel();
    low_state_sub_->InitChannel([this](const void *msg) { callback(static_cast<const u2_msg::LowState_ *>(msg)); }, 1);
  }

  void setSend(const LowCmd &cmd_msg) override {
    for (std::size_t i{}; i < getDoF(); ++i) {
      const auto motor_index = Spec::motorIndex(i);
      const float direction = Spec::motorDirection(i);
      low_cmd_.motor_cmd()[motor_index].q()   = direction * cmd_msg[i].q;
      low_cmd_.motor_cmd()[motor_index].dq()  = direction * cmd_msg[i].dq;
      low_cmd_.motor_cmd()[motor_index].kp()  = cmd_msg[i].Kp;
      low_cmd_.motor_cmd()[motor_index].kd()  = cmd_msg[i].Kd;
      low_cmd_.motor_cmd()[motor_index].tau() = direction * cmd_msg[i].tor;
    }
  }

  void getRecv(LowState &state_msg) override {
    std::lock_guard<std::mutex> _(mutex_);
    state_msg = low_state_;
  }

  void send() override {
    fillLowCmdCrc(low_cmd_);
    low_cmd_pub_->Write(low_cmd_);
  }

  void recv() override {}

 private:
  void callback(const u2_msg::LowState_ *msg) {
    std::lock_guard<std::mutex> _(mutex_);
    low_state_.imu.rpy           = msg->imu_state().rpy();
    low_state_.imu.quaternion    = msg->imu_state().quaternion();
    low_state_.imu.accelerometer = msg->imu_state().accelerometer();
    low_state_.imu.gyroscope     = msg->imu_state().gyroscope();

    for (std::size_t i{}; i < getDoF(); ++i) {
      const auto motor_index = Spec::motorIndex(i);
      const float direction = Spec::motorDirection(i);
      low_state_.motor_state[i].q   = direction * msg->motor_state()[motor_index].q();
      low_state_.motor_state[i].dq  = direction * msg->motor_state()[motor_index].dq();
      low_state_.motor_state[i].tor = direction * msg->motor_state()[motor_index].tau_est();
    }
    for (std::size_t i{}; i < getNumLegs(); ++i) {
      low_state_.foot_force[i] = msg->foot_force()[i];
    }
    low_state_.tick = msg->tick();
  }

  u2_sdk::ChannelPublisherPtr<u2_msg::LowCmd_> low_cmd_pub_;
  u2_sdk::ChannelSubscriberPtr<u2_msg::LowState_> low_state_sub_;
  u2_msg::LowCmd_ low_cmd_{};
  LowState low_state_;
  std::mutex mutex_;
};

struct IdentityQuadrupedMapping {
  static std::size_t motorIndex(std::size_t joint_index) { return joint_index; }
  static float motorDirection(std::size_t) { return 1.0F; }
};

struct Go2Spec : IdentityQuadrupedMapping {
  static const char *robotName() { return "go2"; }
  static constexpr uint8_t kMotorServoMode = kGo2MotorServoMode;
};

struct Go2WSpec : IdentityQuadrupedMapping {
  static const char *robotName() { return "go2w"; }
  static constexpr uint8_t kMotorServoMode = kGo2MotorServoMode;
};

struct B2Spec : IdentityQuadrupedMapping {
  static const char *robotName() { return "b2"; }
  static constexpr uint8_t kMotorServoMode = kB2MotorServoMode;
};

struct B2WSpec {
  static const char *robotName() { return "b2w"; }
  static constexpr uint8_t kMotorServoMode = kB2MotorServoMode;
  // Unitree B2W low-level DDS order: 12 leg joints followed by FR/FL/RR/RL wheels.
  // Keep this table explicit so a hardware revision can be adapted without
  // changing the policy's canonical joint order.
  static constexpr std::array<std::size_t, 16> kMotorIndices{
      0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
  static constexpr std::array<float, 16> kMotorDirections{
      1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F,
      1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F};
  static std::size_t motorIndex(std::size_t joint_index) { return kMotorIndices.at(joint_index); }
  static float motorDirection(std::size_t joint_index) { return kMotorDirections.at(joint_index); }
};

struct AliengoSimSpec : IdentityQuadrupedMapping {
  static const char *robotName() { return "aliengo_sim"; }
  static constexpr uint8_t kMotorServoMode = kGo2MotorServoMode;
};

using Go2Api = UnitreeQuadrupedApi<Go2Spec>;
using Go2WApi = UnitreeQuadrupedApi<Go2WSpec>;
using B2Api = UnitreeQuadrupedApi<B2Spec>;
using B2WApi = UnitreeQuadrupedApi<B2WSpec>;
using AliengoSimApi = UnitreeQuadrupedApi<AliengoSimSpec>;
}  // namespace stepit

#endif  // STEPIT_ROBOT_UNITREE2_QUADRUPED_H_
