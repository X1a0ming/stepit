#ifndef STEPIT_SAFETY_H_
#define STEPIT_SAFETY_H_

#include <cstddef>
#include <vector>

#include <stepit/utils.h>

namespace stepit {
struct LowCmd;
struct LowState;

struct OrientationLimit {
  /** Whether to freeze the robot when its orientation exceeds the configured limits. */
  bool enabled{true};
  /** Maximum allowed roll angle in radians. */
  float roll{M_PIf / 2};
  /** Maximum allowed pitch angle in radians. */
  float pitch{M_PIf / 2};

  /** Loads the orientation-limit configuration. */
  void load(const yml::Node &config);

  /** Returns whether the enabled limit is violated by the robot orientation. */
  bool isViolated(const LowState &state) const;
};

struct NegativeJointPowerLimit {
  /** Whether to enforce the negative joint-power constraint during active output. */
  bool enabled{false};
  /** Maximum allowed negative mechanical power for each joint, in watts. */
  std::vector<float> limits;

  /** Loads and validates the limiter configuration for a robot with the given number of joints. */
  void load(const yml::Node &config, std::size_t dof);

  /**
   * Projects modeled PD command torque onto the configured per-joint negative-power limits.
   *
   * The modeled torque is `tau_ff + kp * (q_des - q) + kd * (dq_des - dq)`. When its mechanical power is below
   * `-limit`, the filter first reduces any opposing feedforward torque toward zero, then changes target position
   * if more correction is needed. If `kp` is unusable, feedforward torque is the final fallback.
   *
   * @pre State and command values are finite.
   * @return Whether at least one joint command was modified.
   */
  bool apply(const LowState &state, const LowCmd &requested_cmd, LowCmd &filtered_cmd) const;
};

struct Safety {
  OrientationLimit orientation_limit;
  NegativeJointPowerLimit negative_joint_power_limit;

  /** Loads all robot safety configuration. */
  void load(const yml::Node &config, std::size_t dof);
};
}  // namespace stepit

#endif  // STEPIT_SAFETY_H_
