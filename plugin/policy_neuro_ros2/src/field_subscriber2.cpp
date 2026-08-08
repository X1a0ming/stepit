#include <stepit/policy_neuro_ros2/field_subscriber2.h>
#include <stepit/ros2/node.h>

namespace stepit::neuro_policy {
FieldSubscriber2::FieldSubscriber2(const NeuroPolicySpec &policy_spec, const ModuleSpec &module_spec)
    : Module(policy_spec, ModuleSpec(module_spec, "field_subscriber")) {
  config_.assertMap();
  using std_msgs::msg::Float32MultiArray;
  for (const auto &field_node : config_) {
    FieldData field;
    field_node.first.to(field.name);
    field_node.second["topic"].to(field.topic);
    field_node.second["size"].to(field.size);
    field_node.second["timeout_threshold"].to(field.timeout_threshold, true);
    field_node.second["bootstrap_zero"].to(field.bootstrap_zero, true);
    const auto expected_layout = field_node.second["expected_layout"];
    if (expected_layout.hasValue()) {
      expected_layout.assertMap();
      expected_layout["labels"].to(field.expected_layout_labels);
      expected_layout["sizes"].to(field.expected_layout_sizes);
      expected_layout["strides"].to(field.expected_layout_strides);
      expected_layout["data_offset"].to(field.expected_data_offset, true);
      expected_layout.require(
          not field.expected_layout_labels.empty() and
              field.expected_layout_labels.size() == field.expected_layout_sizes.size() and
              field.expected_layout_labels.size() == field.expected_layout_strides.size(),
          "'expected_layout' labels, sizes, and strides must have the same non-zero length");
      std::size_t element_count = 1;
      for (const auto size : field.expected_layout_sizes) element_count *= size;
      expected_layout.require(element_count == field.size,
                              "'expected_layout' sizes must multiply to field size");
      field.validate_layout = true;
    }
    field.id   = registerProvision(field.name, field.size);
    field.data = VecXf::Zero(static_cast<Eigen::Index>(field.size));

    std::size_t index = fields_.size();
    rclcpp::QoS qos   = parseQoS(field_node.second["qos"]);
    field.subscriber  = getNode()->create_subscription<Float32MultiArray>(
        field.topic, qos, [this, index](const Float32MultiArray::SharedPtr msg) { callback(index, msg); });
    field.received = false;
    fields_.push_back(std::move(field));
  }
}

bool FieldSubscriber2::reset() {
  std::lock_guard<std::mutex> _(mutex_);
  for (auto &field : fields_) {
    if (field.bootstrap_zero and not field.received) {
      field.data.setZero();
      field.stamp = getNode()->now();
      field.received = true;
    }
    if (not field.received) {
      STEPIT_WARN("Field '{}' is not received yet.", field.name);
      return false;
    }
  }
  return true;
}

bool FieldSubscriber2::update(const LowState &low_state, ControlRequests &requests, FieldMap &context) {
  std::lock_guard<std::mutex> _(mutex_);
  for (const auto &field : fields_) {
    if (not field.received) return false;
    if (field.timeout_threshold > 0.0 and getElapsedTime(field.stamp) > field.timeout_threshold) {
      STEPIT_WARN("Field '{}' has timed out.", field.name);
      return false;
    }
    if (field.data.size() != static_cast<Eigen::Index>(field.size)) {
      STEPIT_WARN("Field '{}' has unexpected size: expected {}, got {}.", field.name, field.size, field.data.size());
      return false;
    }
    context[field.id] = field.data;
  }
  return true;
}

void FieldSubscriber2::callback(std::size_t index, const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  std::lock_guard<std::mutex> _(mutex_);
  if (index >= fields_.size()) return;
  auto &field = fields_[index];
  if (msg->data.size() != field.size) {
    field.received = false;
    STEPIT_WARN("Field '{}' rejected data of size {}; expected {}.", field.name, msg->data.size(), field.size);
    return;
  }
  if (field.validate_layout) {
    bool matches = msg->layout.data_offset == field.expected_data_offset and
                   msg->layout.dim.size() == field.expected_layout_labels.size();
    for (std::size_t i = 0; matches and i < msg->layout.dim.size(); ++i) {
      const auto &actual = msg->layout.dim[i];
      matches = actual.label == field.expected_layout_labels[i] and
                actual.size == field.expected_layout_sizes[i] and
                actual.stride == field.expected_layout_strides[i];
    }
    if (not matches) {
      field.received = false;
      STEPIT_WARN("Field '{}' rejected Float32MultiArray with an unexpected layout.", field.name);
      return;
    }
  }
  field.received = true;
  field.stamp    = getNode()->now();
  field.data     = VecXf::Map(msg->data.data(), static_cast<Eigen::Index>(msg->data.size()));
}

STEPIT_REGISTER_MODULE(field_subscriber, kDefPriority, Module::make<FieldSubscriber2>);
}  // namespace stepit::neuro_policy
