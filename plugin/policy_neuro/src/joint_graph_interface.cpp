#include <algorithm>
#include <cmath>
#include <numeric>
#include <set>

#include <stepit/policy_neuro/joint_graph_interface.h>

namespace stepit::neuro_policy {
namespace {

ArrXf requiredFiniteArray(const yml::Node &node, FieldSize size, const std::string &label) {
  node.assertSequence(size);
  ArrXf value{static_cast<Eigen::Index>(size)};
  node.to(value);
  node.require(value.allFinite(), fmt::format("'{}' must contain only finite values", label));
  return value;
}

ArrXf optionalFiniteArray(const yml::Node &node, FieldSize size, float default_value,
                          const std::string &label) {
  ArrXf value = ArrXf::Constant(static_cast<Eigen::Index>(size), default_value);
  if (node.hasValue()) {
    node.assertSequence(size);
    node.to(value);
    node.require(value.allFinite(), fmt::format("'{}' must contain only finite values", label));
  }
  return value;
}

}  // namespace

JointGraphObservation::JointGraphObservation(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : Module(policy_spec, ModuleSpec(module_spec, "joint_graph_observation")) {
  static_assert(kJointFeaturesOffset + kMaxJoints * kJointFeatureSize == kParentOffset);
  static_assert(kParentOffset + kMaxJoints == kControlModeOffset);
  static_assert(kControlModeOffset + kMaxJoints == kJointValidOffset);
  static_assert(kJointValidOffset + kMaxJoints == kObservationSize);

  config_.assertMap();
  config_["robot_id"].to(robot_id_);
  config_["robot_id"].require(robot_id_ <= 8, "'robot_id' must be in [0, 8]");
  config_["cmd_active"].to(cmd_active_);
  config_["motor_active"].to(motor_active_);
  config_.require(cmd_active_, "joint_graph_v5 requires the current command branch to be active");
  config_.require(motor_active_, "Stepit motor deployment requires the graph motor branch to be active");

  config_["joint_count"].to(joint_count_);
  config_["joint_count"].require(joint_count_ > 0 and joint_count_ <= kMaxJoints,
                                  "'joint_count' must be in [1, 29]");
  config_["joint_count"].require(joint_count_ == policy_spec.dof,
                                  "joint_count must equal the Stepit robot DoF");

  const auto parent_node = config_["parent_indices"];
  const auto mode_node = config_["control_modes"];
  parent_node.assertSequence(joint_count_);
  mode_node.assertSequence(joint_count_);
  parent_node.to(parent_indices_);
  mode_node.to(control_modes_);
  validateTopology(parent_node);
  for (std::size_t index = 0; index < control_modes_.size(); ++index) {
    mode_node.require(control_modes_[index] == 1 or control_modes_[index] == 2,
                      fmt::format("control mode at slot {} must be 1 (position) or 2 (velocity)", index));
  }

  const auto joint_indices_node = config_["joint_indices"];
  if (joint_indices_node.hasValue()) {
    joint_indices_node.assertSequence(joint_count_);
    joint_indices_node.to(joint_indices_);
  } else {
    joint_indices_.resize(joint_count_);
    std::iota(joint_indices_.begin(), joint_indices_.end(), 0);
  }
  std::set<std::size_t> unique_indices;
  for (const auto index : joint_indices_) {
    joint_indices_node.require(index < policy_spec.dof, "'joint_indices' contains an out-of-range source index");
    joint_indices_node.require(unique_indices.insert(index).second, "'joint_indices' contains a duplicate index");
  }

  const auto features = config_["joint_features"];
  features.assertMap();
  default_position_ = requiredFiniteArray(features["default_position"], joint_count_, "default_position");
  default_velocity_ = optionalFiniteArray(features["default_velocity"], joint_count_, 0.0F, "default_velocity");
  lower_limit_ = requiredFiniteArray(features["lower_limit"], joint_count_, "lower_limit");
  upper_limit_ = requiredFiniteArray(features["upper_limit"], joint_count_, "upper_limit");
  velocity_limit_ = requiredFiniteArray(features["velocity_limit"], joint_count_, "velocity_limit");
  effort_limit_ = requiredFiniteArray(features["effort_limit"], joint_count_, "effort_limit");
  stiffness_ = requiredFiniteArray(features["stiffness"], joint_count_, "stiffness");
  damping_ = requiredFiniteArray(features["damping"], joint_count_, "damping");
  action_scale_ = requiredFiniteArray(features["action_scale"], joint_count_, "action_scale");
  features.require((lower_limit_ <= upper_limit_).all(), "every lower_limit must be <= upper_limit");
  features.require((velocity_limit_ >= 0.0F).all(), "velocity_limit must be non-negative");
  features.require((effort_limit_ >= 0.0F).all(), "effort_limit must be non-negative");
  features.require((stiffness_ >= 0.0F).all(), "stiffness must be non-negative");
  features.require((damping_ >= 0.0F).all(), "damping must be non-negative");

  const auto scales = config_["scales"];
  scales["ang_vel"].to(ang_vel_scale_, true);
  scales["gravity"].to(gravity_scale_, true);
  scales["path"].to(path_scale_, true);
  scales["previous_cmd_vel"].to(previous_cmd_vel_scale_, true);
  scales["joint_pos"].to(joint_pos_scale_, true);
  scales["joint_vel"].to(joint_vel_scale_, true);
  scales.require(std::isfinite(ang_vel_scale_) and std::isfinite(gravity_scale_) and
                     std::isfinite(path_scale_) and std::isfinite(previous_cmd_vel_scale_) and
                     std::isfinite(joint_pos_scale_) and std::isfinite(joint_vel_scale_),
                 "all observation scales must be finite");
  cmd_vel_clip_lower_ = requiredFiniteArray(config_["cmd_vel_clip_lower"], kCmdVelSize,
                                             "cmd_vel_clip_lower");
  cmd_vel_clip_upper_ = requiredFiniteArray(config_["cmd_vel_clip_upper"], kCmdVelSize,
                                             "cmd_vel_clip_upper");
  config_.require((cmd_vel_clip_lower_ <= cmd_vel_clip_upper_).all(),
                  "every cmd_vel clip lower bound must be <= its upper bound");

  const auto fields = config_["fields"];
  ang_vel_id_ = registerRequirement(fields["ang_vel"].as<std::string>("ang_vel"), 3);
  gravity_id_ = registerRequirement(fields["gravity"].as<std::string>("gravity"), 3);
  path_id_ = registerRequirement(fields["path"].as<std::string>("path_points_body"), kPathSize);
  joint_pos_id_ = registerRequirement(fields["joint_pos"].as<std::string>("joint_pos"), policy_spec.dof);
  joint_vel_id_ = registerRequirement(fields["joint_vel"].as<std::string>("joint_vel"), policy_spec.dof);
  observation_id_ = registerProvision(fields["observation"].as<std::string>("joint_graph_observation"),
                                       kObservationSize);

  // These IDs are not current-step requirements. postStep() reads the
  // completed actor context and stores history for the next observation.
  graph_cmd_vel_id_ = registerField(fields["graph_cmd_vel"].as<std::string>("joint_graph_cmd_vel"),
                                     kCmdVelSize);
  graph_motor_id_ = registerField(fields["graph_motor"].as<std::string>("joint_graph_motor"), kMaxJoints);
}

float JointGraphObservation::encodePositiveLimit(float value, float maximum) {
  return std::log1p(std::min(std::abs(value), maximum)) / std::log1p(maximum);
}

void JointGraphObservation::validateTopology(const yml::Node &parent_node) const {
  for (std::size_t start = 0; start < parent_indices_.size(); ++start) {
    std::set<int> visited;
    int cursor = static_cast<int>(start);
    while (cursor != -1) {
      parent_node.require(cursor >= 0 and cursor < static_cast<int>(joint_count_),
                          fmt::format("parent chain from slot {} contains out-of-range slot {}", start, cursor));
      parent_node.require(visited.insert(cursor).second,
                          fmt::format("joint topology contains a cycle through slot {}", cursor));
      cursor = parent_indices_[static_cast<std::size_t>(cursor)];
    }
  }
}

bool JointGraphObservation::reset() {
  previous_cmd_vel_.setZero();
  previous_motor_action_.setZero();
  return true;
}

bool JointGraphObservation::update(const LowState &, ControlRequests &, FieldMap &context) {
  const auto &ang_vel = context.at(ang_vel_id_);
  const auto &gravity = context.at(gravity_id_);
  const auto &path = context.at(path_id_);
  const auto &joint_pos = context.at(joint_pos_id_);
  const auto &joint_vel = context.at(joint_vel_id_);
  if (not ang_vel.allFinite() or not gravity.allFinite() or not path.allFinite() or
      not joint_pos.allFinite() or not joint_vel.allFinite()) {
    STEPIT_WARN("Joint-graph input contains non-finite values.");
    return false;
  }

  observation_.setZero();
  observation_.segment(kParentOffset, kMaxJoints).setConstant(-1.0F);
  observation_.segment(kAngVelOffset, 3) = ang_vel * ang_vel_scale_;
  observation_.segment(kGravityOffset, 3) = gravity * gravity_scale_;
  observation_.segment(kPathOffset, kPathSize) = path * path_scale_;
  observation_.segment(kPreviousCmdVelOffset, kCmdVelSize) = previous_cmd_vel_ * previous_cmd_vel_scale_;
  observation_[kRobotIdOffset] = static_cast<float>(robot_id_);
  observation_[kCmdActiveOffset] = 1.0F;
  observation_[kMotorActiveOffset] = 1.0F;

  for (FieldSize slot = 0; slot < joint_count_; ++slot) {
    const auto source = static_cast<Eigen::Index>(joint_indices_[slot]);
    const auto joint = static_cast<Eigen::Index>(slot);
    const FieldSize offset = kJointFeaturesOffset + slot * kJointFeatureSize;
    observation_[offset + 0] = (joint_pos[source] - default_position_[joint]) * joint_pos_scale_;
    observation_[offset + 1] = (joint_vel[source] - default_velocity_[joint]) * joint_vel_scale_;
    observation_[offset + 2] = previous_motor_action_[joint];
    observation_[offset + 3] = default_position_[joint];
    observation_[offset + 4] = std::clamp(lower_limit_[joint], -10.0F, 10.0F);
    observation_[offset + 5] = std::clamp(upper_limit_[joint], -10.0F, 10.0F);
    observation_[offset + 6] = encodePositiveLimit(velocity_limit_[joint], 100.0F);
    observation_[offset + 7] = encodePositiveLimit(effort_limit_[joint], 1000.0F);
    observation_[offset + 8] = encodePositiveLimit(stiffness_[joint], 1000.0F);
    observation_[offset + 9] = encodePositiveLimit(damping_[joint], 100.0F);
    observation_[offset + 10] = std::clamp(action_scale_[joint], -10.0F, 10.0F);
    observation_[kParentOffset + slot] = static_cast<float>(parent_indices_[slot]);
    observation_[kControlModeOffset + slot] = static_cast<float>(control_modes_[slot]);
    observation_[kJointValidOffset + slot] = 1.0F;
  }

  if (not observation_.allFinite()) {
    STEPIT_WARN("Joint-graph observation builder produced non-finite values.");
    return false;
  }
  context[observation_id_] = observation_;
  return true;
}

void JointGraphObservation::postStep(const FieldMap &context) {
  const auto raw_cmd = context.find(graph_cmd_vel_id_);
  STEPIT_ASSERT(raw_cmd != context.end() and raw_cmd->second.size() == kCmdVelSize,
                "Joint-graph actor must provide a 3-D cmd_vel field.");
  STEPIT_ASSERT(raw_cmd->second.allFinite(), "Joint-graph cmd_vel output must be finite.");
  previous_cmd_vel_ = raw_cmd->second.cwiseMax(cmd_vel_clip_lower_).cwiseMin(cmd_vel_clip_upper_);

  const auto motor = context.find(graph_motor_id_);
  STEPIT_ASSERT(motor != context.end() and motor->second.size() == kMaxJoints,
                "Joint-graph actor must provide a 29-D motor field.");
  STEPIT_ASSERT(motor->second.allFinite(), "Joint-graph motor output must be finite.");
  previous_motor_action_.setZero();
  previous_motor_action_.head(joint_count_) = motor->second.head(joint_count_);
}

STEPIT_REGISTER_MODULE(joint_graph_observation, kDefPriority, Module::make<JointGraphObservation>);
STEPIT_REGISTER_FIELD_SOURCE(joint_graph_observation, kDefPriority, Module::make<JointGraphObservation>);

}  // namespace stepit::neuro_policy
