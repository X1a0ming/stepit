#ifndef STEPIT_POLICY_H_
#define STEPIT_POLICY_H_

#include <stepit/communication.h>
#include <stepit/control_input.h>
#include <stepit/robot.h>

namespace stepit {
struct PolicySpec : RobotSpec {
  explicit PolicySpec(const RobotSpec &robot_spec) : RobotSpec(robot_spec) {}

  /* Name of the policy */
  std::string policy_name;
  /* Control frequency in Hz */
  std::size_t control_freq{};
  /* Whether the policy is trusted in the case of safety violations */
  bool trusted{};
};

class Policy {
 public:
  using Ptr = std::unique_ptr<Policy>;
  using Reg = RegistrySingleton<Policy, const RobotSpec & /* robot_spec */, const std::string & /* home_dir */>;

  explicit Policy(const RobotSpec &spec) : spec_(spec) {}
  virtual ~Policy() = default;

  std::size_t getControlFreq() const { return spec_.control_freq; }
  std::string getName() const { return spec_.policy_name; }
  bool isTrusted() const { return spec_.trusted; }
  const PolicySpec &getSpec() const { return spec_; }

  virtual bool reset()                                                                = 0;
  virtual bool act(const LowState &low_state, ControlRequests &requests, LowCmd &cmd) = 0;
  virtual void exit()                                                                 = 0;

 protected:
  PolicySpec spec_;
};

using PolicyPtr = Policy::Ptr;
using PolicyReg = Policy::Reg;
extern template class RegistrySingleton<Policy, const RobotSpec &, const std::string &>;
}  // namespace stepit

#define STEPIT_REGISTER_POLICY(name, priority, factory) \
  static ::stepit::PolicyReg::Registration _policy_##name##_registration(#name, priority, factory)

#endif  // STEPIT_POLICY_H_
