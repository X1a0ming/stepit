#ifndef STEPIT_ROBOT_UNITREE2_ROS2_ROBOT_API_H_
#define STEPIT_ROBOT_UNITREE2_ROS2_ROBOT_API_H_

#include <mutex>

#include <rclcpp/publisher.hpp>
#include <unitree_go/msg/low_cmd.hpp>
#include <unitree_go/msg/low_state.hpp>

#include <stepit/robot.h>
#include <stepit/ros2/node.h>

namespace u2ros2_msg = unitree_go::msg;

namespace stepit {
class Unitree2Ros2Quadruped final : public RobotApi {
 public:
  Unitree2Ros2Quadruped(const std::string &robot_name);
  void getControl(bool enable) override;
  void setSend(LowCmd &cmd_msg) override;
  void getRecv(LowState &state_msg) override;
  void send() override;
  void recv() override {}

  static constexpr std::size_t kDoF     = 12;
  static constexpr std::size_t kNumLegs = 4;
  std::size_t getDoF() const override { return kDoF; }
  std::size_t getNumLegs() const override { return kNumLegs; }
  std::size_t getCommFreq() const override { return 500; }

 private:
  void callback(const u2ros2_msg::LowState::SharedPtr msg);

  rclcpp::Publisher<u2ros2_msg::LowCmd>::SharedPtr low_cmd_pub_;
  rclcpp::Subscription<u2ros2_msg::LowState>::SharedPtr low_state_sub_;
  u2ros2_msg::LowCmd low_cmd_{};
  LowState low_state_;
  std::mutex mutex_;
};
}  // namespace stepit

#endif  // STEPIT_ROBOT_UNITREE2_ROS2_ROBOT_API_H_
