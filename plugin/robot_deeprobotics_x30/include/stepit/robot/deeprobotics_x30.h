#ifndef STEPIT_ROBOT_DEEPROBOTICS_X30_H_
#define STEPIT_ROBOT_DEEPROBOTICS_X30_H_

#include <parse_cmd.h>
#include <send_to_robot.h>

#include <stepit/robot.h>

namespace stepit {
class DeepRoboticsX30BaseApi : public RobotApi {
 public:
  DeepRoboticsX30BaseApi(const std::string &name);
  void getControl(bool enable) override;
  void send() override {}
  void recv() override {}

  static constexpr std::size_t kDoF     = 12;
  static constexpr std::size_t kNumLegs = 4;
  std::size_t getDoF() const override { return kDoF; }
  std::size_t getNumLegs() const override { return kNumLegs; }
  std::size_t getCommFreq() const override { return 1000; }

 protected:
  std::unique_ptr<x30::SendToRobot> low_cmd_pub_;
  x30::ParseCommand low_state_sub_;
  x30::RobotDataSDK *state_msg_{nullptr};
  x30::RobotCmdSDK cmd_msg_{};
};

class DeepRoboticsX30Api final : public DeepRoboticsX30BaseApi {
 public:
  DeepRoboticsX30Api();
  void setSend(LowCmd &cmd_msg) override;
  void getRecv(LowState &state_msg) override;
  static constexpr const char *kRobotName = "x30";
};

class DeepRoboticsX30uApi final : public DeepRoboticsX30BaseApi {
 public:
  DeepRoboticsX30uApi();
  void setSend(LowCmd &cmd_msg) override;
  void getRecv(LowState &state_msg) override;
  static constexpr const char *kRobotName = "x30u";
  static constexpr std::size_t kJointOrder[]{3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};
  static constexpr std::size_t kFootOrder[]{1, 0, 3, 2};
};
}  // namespace stepit

#endif  // STEPIT_ROBOT_DEEPROBOTICS_X30_H_
