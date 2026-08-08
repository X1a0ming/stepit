#ifndef STEPIT_NEURO_POLICY_JOINT_GRAPH_INTERFACE_H_
#define STEPIT_NEURO_POLICY_JOINT_GRAPH_INTERFACE_H_

#include <cstdint>
#include <string>
#include <vector>

#include <stepit/policy_neuro/module.h>

namespace stepit::neuro_policy {

/** Assemble the fixed MIRLab joint_graph_v5 no-height observation.
 *
 * Layout (458 floats):
 *   ang_vel(3), gravity(3), path_xy(40), previous_cmd_vel(3),
 *   robot_id(1), cmd_active(1), motor_active(1),
 *   joint_features(29 * 11, joint-major), parent(29), mode(29), valid(29).
 *
 * The module owns the previous graph-output history. It reads the actor's
 * completed output in postStep(), avoiding a current-step dependency cycle.
 */
class JointGraphObservation : public Module {
 public:
  static constexpr FieldSize kObservationSize = 458;
  static constexpr FieldSize kPathSize = 40;
  static constexpr FieldSize kCmdVelSize = 3;
  static constexpr FieldSize kMaxJoints = 29;
  static constexpr FieldSize kJointFeatureSize = 11;
  static constexpr FieldSize kGraphActionSize = 32;

  static constexpr FieldSize kAngVelOffset = 0;
  static constexpr FieldSize kGravityOffset = 3;
  static constexpr FieldSize kPathOffset = 6;
  static constexpr FieldSize kPreviousCmdVelOffset = 46;
  static constexpr FieldSize kRobotIdOffset = 49;
  static constexpr FieldSize kCmdActiveOffset = 50;
  static constexpr FieldSize kMotorActiveOffset = 51;
  static constexpr FieldSize kJointFeaturesOffset = 52;
  static constexpr FieldSize kParentOffset = 371;
  static constexpr FieldSize kControlModeOffset = 400;
  static constexpr FieldSize kJointValidOffset = 429;

  JointGraphObservation(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec);

  bool reset() override;
  bool update(const LowState &low_state, ControlRequests &requests, FieldMap &context) override;
  void postStep(const FieldMap &context) override;

 private:
  static float encodePositiveLimit(float value, float maximum);
  void validateTopology(const yml::Node &parent_node) const;

  bool cmd_active_{};
  bool motor_active_{};
  std::uint32_t robot_id_{};
  FieldSize joint_count_{};
  std::vector<std::size_t> joint_indices_;
  std::vector<int> parent_indices_;
  std::vector<int> control_modes_;

  ArrXf default_position_;
  ArrXf default_velocity_;
  ArrXf lower_limit_;
  ArrXf upper_limit_;
  ArrXf velocity_limit_;
  ArrXf effort_limit_;
  ArrXf stiffness_;
  ArrXf damping_;
  ArrXf action_scale_;

  float ang_vel_scale_{1.0F};
  float gravity_scale_{1.0F};
  float path_scale_{1.0F};
  float previous_cmd_vel_scale_{1.0F};
  float joint_pos_scale_{1.0F};
  float joint_vel_scale_{1.0F};
  ArrXf cmd_vel_clip_lower_{ArrXf::Constant(kCmdVelSize, -2.0F)};
  ArrXf cmd_vel_clip_upper_{ArrXf::Constant(kCmdVelSize, 2.0F)};

  FieldId ang_vel_id_{};
  FieldId gravity_id_{};
  FieldId path_id_{};
  FieldId joint_pos_id_{};
  FieldId joint_vel_id_{};
  FieldId observation_id_{};
  FieldId graph_cmd_vel_id_{};
  FieldId graph_motor_id_{};

  ArrXf observation_{ArrXf::Zero(kObservationSize)};
  ArrXf previous_cmd_vel_{ArrXf::Zero(kCmdVelSize)};
  ArrXf previous_motor_action_{ArrXf::Zero(kMaxJoints)};
};

}  // namespace stepit::neuro_policy

#endif  // STEPIT_NEURO_POLICY_JOINT_GRAPH_INTERFACE_H_
