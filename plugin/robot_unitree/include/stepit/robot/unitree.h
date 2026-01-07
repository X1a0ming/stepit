#ifndef STEPIT_ROBOT_UNITREE_H_
#define STEPIT_ROBOT_UNITREE_H_

#include <unitree_legged_sdk/unitree_legged_sdk.h>

#include <stepit/robot.h>

namespace stepit {
namespace unitree = UNITREE_LEGGED_SDK;

class UnitreeApi final : public RobotApi {
 public:
  UnitreeApi();
  void getControl(bool enable) override;
  void setSend(LowCmd &cmd_msg) override;
  void getRecv(LowState &state_msg) override;
  void send() override;
  void recv() override;

  static constexpr std::size_t kDoF     = 12;
  static constexpr std::size_t kNumLegs = 4;
  static constexpr const char *getRobotName();
  std::size_t getDoF() const override { return kDoF; }
  std::size_t getNumLegs() const override { return kNumLegs; }
  std::size_t getCommFreq() const override { return 1000; }

 private:
  unitree::UDP udp_;
  unitree::LowCmd low_cmd_{};
  unitree::LowState low_state_{};
};
}  // namespace stepit

#endif  // STEPIT_ROBOT_UNITREE_H_
