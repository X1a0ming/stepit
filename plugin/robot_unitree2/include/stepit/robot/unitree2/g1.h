#ifndef STEPIT_ROBOT_UNITREE2_G1_H_
#define STEPIT_ROBOT_UNITREE2_G1_H_

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <stepit/robot.h>
#include <stepit/utils.h>
#include <stepit/robot/unitree2/common.h>

namespace hg_msg = unitree_hg::msg::dds_;

namespace stepit {
class G1DoF15Api : public RobotApi {
 public:
  G1DoF15Api();
  void getControl(bool enable) override;
  void setSend(LowCmd &cmd_msg) override;
  void getRecv(LowState &state_msg) override;
  void send() override;
  void recv() override {}

  static constexpr std::size_t kDoF       = 15;
  static constexpr std::size_t kArmDoF    = 14;
  static constexpr std::size_t kNumLegs   = 2;
  static constexpr const char *kRobotName = "g1_15dof";
  std::size_t getDoF() const override { return kDoF; }
  std::size_t getNumLegs() const override { return kNumLegs; }
  std::size_t getCommFreq() const override { return 500; }

 private:
  void callback(const hg_msg::LowState_ *msg);

 protected:
  virtual void setArmCommands();

  u2_sdk::ChannelPublisherPtr<hg_msg::LowCmd_> low_cmd_pub_;
  u2_sdk::ChannelSubscriberPtr<hg_msg::LowState_> low_state_sub_;
  hg_msg::LowCmd_ low_cmd_{};
  LowState low_state_;
  std::mutex mutex_;

  ArrXf default_arm_pos_{kArmDoF};
  std::array<MotorState, kArmDoF> arm_state_{};
  LowCmd arm_cmd_{kArmDoF};
};

class G1DoF23Api final : public RobotApi {
 public:
  G1DoF23Api();
  void getControl(bool enable) override;
  void setSend(LowCmd &cmd_msg) override;
  void getRecv(LowState &state_msg) override;
  void send() override;
  void recv() override {}

  static constexpr std::size_t kDoF       = 23;
  static constexpr std::size_t kNumLegs   = 2;
  static constexpr const char *kRobotName = "g1_23dof";
  std::size_t getDoF() const override { return kDoF; }
  std::size_t getNumLegs() const override { return kNumLegs; }
  std::size_t getCommFreq() const override { return 500; }

 private:
  void callback(const hg_msg::LowState_ *msg);

  u2_sdk::ChannelPublisherPtr<hg_msg::LowCmd_> low_cmd_pub_;
  u2_sdk::ChannelSubscriberPtr<hg_msg::LowState_> low_state_sub_;
  hg_msg::LowCmd_ low_cmd_{};
  LowState low_state_;
  std::mutex mutex_;

  ArrXf curr_arm_pos_{6};
  ArrXf des_arm_pos_{6};
};

class G1DoF29Api final : public RobotApi {
 public:
  G1DoF29Api();
  void getControl(bool enable) override;
  void setSend(LowCmd &cmd_msg) override;
  void getRecv(LowState &state_msg) override;
  void send() override;
  void recv() override {}

  static constexpr std::size_t kDoF       = 29;
  static constexpr std::size_t kNumLegs   = 2;
  static constexpr const char *kRobotName = "g1_29dof";
  std::size_t getDoF() const override { return kDoF; }
  std::size_t getNumLegs() const override { return kNumLegs; }
  std::size_t getCommFreq() const override { return 500; }

 private:
  void callback(const hg_msg::LowState_ *msg);

  u2_sdk::ChannelPublisherPtr<hg_msg::LowCmd_> low_cmd_pub_;
  u2_sdk::ChannelSubscriberPtr<hg_msg::LowState_> low_state_sub_;
  hg_msg::LowCmd_ low_cmd_{};
  LowState low_state_;
  std::mutex mutex_;
};

}  // namespace stepit

#endif  // STEPIT_ROBOT_UNITREE2_G1_H_
