#ifndef STEPIT_ROBOT_DEEPROBOTICS_LITE3_H_
#define STEPIT_ROBOT_DEEPROBOTICS_LITE3_H_

#include <atomic>

#include <receiver.h>
#include <sender.h>

#include <stepit/robot.h>

namespace stepit {
class DeepRoboticsLite3Api final : public RobotApi {
 public:
  DeepRoboticsLite3Api();
  ~DeepRoboticsLite3Api() override;
  void getControl(bool enable) override;
  void setSend(LowCmd &cmd_msg) override;
  void getRecv(LowState &state_msg) override;
  void send() override {}
  void recv() override {}

  static constexpr std::size_t kDoF       = 12;
  static constexpr std::size_t kNumLegs   = 4;
  static constexpr const char *kRobotName = "lite3";
  std::size_t getDoF() const override { return kDoF; }
  std::size_t getNumLegs() const override { return kNumLegs; }
  std::size_t getCommFreq() const override { return 1000; }

 private:
  std::unique_ptr<Sender> low_cmd_pub_;
  std::unique_ptr<Receiver> low_state_sub_;
  RobotData *state_msg_{nullptr};
  RobotCmd cmd_msg_{};
  std::atomic<std::size_t> tick_{0};
};
}  // namespace stepit

#endif  // STEPIT_ROBOT_DEEPROBOTICS_LITE3_H_
