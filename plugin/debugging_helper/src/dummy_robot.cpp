#include <stepit/debugging/dummy_robot.h>

namespace stepit {
void DummyRobotApi::getControl(bool enable) { low_state_.tick = 1; }

void DummyRobotApi::setSend(LowCmd &cmd) {
  for (std::size_t i{}; i < getDoF(); ++i) {
    low_state_.motor_state[i].q   = cmd[i].q;
    low_state_.motor_state[i].dq  = cmd[i].dq;
    low_state_.motor_state[i].tor = cmd[i].tor;
  }
}

void DummyRobotApi::getRecv(LowState &state) { state = low_state_; }

STEPIT_REGISTER_ROBOTAPI(dummy, kMinPriority, [] { return std::make_unique<DummyRobotApi>(); });
}  // namespace stepit
