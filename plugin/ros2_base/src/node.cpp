#include <stepit/ros2/node.h>

namespace stepit {
std::shared_ptr<rclcpp::Node> g_node;

const std::map<std::string, rclcpp::ReliabilityPolicy> kReliabilityPolicyMap = {
    {"reliable", rclcpp::ReliabilityPolicy::Reliable},
    {"best_effort", rclcpp::ReliabilityPolicy::BestEffort},
    {"system_default", rclcpp::ReliabilityPolicy::SystemDefault},
};

const std::map<std::string, rclcpp::DurabilityPolicy> kDurabilityPolicyMap = {
    {"volatile", rclcpp::DurabilityPolicy::Volatile},
    {"transient_local", rclcpp::DurabilityPolicy::TransientLocal},
    {"system_default", rclcpp::DurabilityPolicy::SystemDefault},
};

const std::map<std::string, rclcpp::HistoryPolicy> kHistoryPolicyMap = {
    {"keep_last", rclcpp::HistoryPolicy::KeepLast},
    {"keep_all", rclcpp::HistoryPolicy::KeepAll},
    {"system_default", rclcpp::HistoryPolicy::SystemDefault},
};

rclcpp::Node::SharedPtr &getNode() { return g_node; }

rclcpp::QoS &getDefaultQoS() {
  static rclcpp::QoS default_qos{rclcpp::SensorDataQoS()};
  static bool initialized = false;
  if (not initialized) {
    std::string reliability, durability, history;
    if (getenv("STEPIT_ROS2_QOS_RELIABILITY", reliability)) {
      toLowercaseInplace(reliability);
      auto reliability_policy = kReliabilityPolicyMap.find(reliability);
      STEPIT_ASSERT(reliability_policy != kReliabilityPolicyMap.end(), "Unknown QoS reliability '{}'.", reliability);
      default_qos.reliability(reliability_policy->second);
    }

    if (getenv("STEPIT_ROS2_QOS_DURABILITY", durability)) {
      toLowercaseInplace(durability);
      auto durability_policy = kDurabilityPolicyMap.find(durability);
      STEPIT_ASSERT(durability_policy != kDurabilityPolicyMap.end(), "Unknown QoS durability '{}'.", durability);
      default_qos.durability(durability_policy->second);
    }

    if (getenv("STEPIT_ROS2_QOS_HISTORY", history)) {
      char *end{nullptr};
      long value = std::strtol(history.c_str(), &end, 10);
      if (*end == '\0') {  // integer
        default_qos.keep_last(static_cast<std::size_t>(value));
      } else {
        toLowercaseInplace(history);
        auto history_policy = kHistoryPolicyMap.find(history);
        STEPIT_ASSERT(history_policy != kHistoryPolicyMap.end(), "Unknown QoS history '{}'.", history);
        default_qos.history(history_policy->second);
      }
    }
    initialized = true;
  }

  return default_qos;
}

std::string getTopicType(const std::string &topic_name, const std::string &default_type) {
  auto topic_info = getNode()->get_topic_names_and_types();
  auto it         = topic_info.find(topic_name);
  if (it == topic_info.end()) return default_type;
  const auto &topic_types = it->second;
  if (topic_types.empty()) return default_type;
  return topic_types[0];
}

rclcpp::QoS parseQoS(const yml::Node &node) {
  rclcpp::QoS value{getDefaultQoS()};
  if (not node) return value;

  if (YAML::Node reliability_node = node["reliability"]) {
    auto reliability = yml::readAs<std::string>(reliability_node);
    toLowercaseInplace(reliability);
    auto reliability_policy = kReliabilityPolicyMap.find(reliability);
    STEPIT_ASSERT(reliability_policy != kReliabilityPolicyMap.end(), "Unknown QoS reliability '{}'.", reliability);
    value.reliability(reliability_policy->second);
  }

  if (YAML::Node durability_node = node["durability"]) {
    auto durability = yml::readAs<std::string>(durability_node);
    toLowercaseInplace(durability);
    auto durability_policy = kDurabilityPolicyMap.find(durability);
    STEPIT_ASSERT(durability_policy != kDurabilityPolicyMap.end(), "Unknown QoS durability '{}'.", durability);
    value.durability(durability_policy->second);
  }

  if (YAML::Node history_node = node["history"]) {
    if (yml::isType<std::size_t>(history_node)) {
      value.keep_last(yml::readIf<std::size_t>(history_node, "history_node", 1));
    } else {
      auto history = yml::readAs<std::string>(history_node);
      toLowercaseInplace(history);
      auto history_policy = kHistoryPolicyMap.find(history);
      STEPIT_ASSERT(history_policy != kHistoryPolicyMap.end(), "Unknown QoS history '{}'.", history);
      value.history(history_policy->second);
    }
  }
  return value;
}

TopicInfo parseTopicInfo(const yml::Node &node, const std::string &default_name, const std::string &default_type) {
  TopicInfo info{};
  if (default_name.empty()) {
    yml::setTo(node, "topic", info.name);
  } else {
    info.name = yml::readIf(node, "topic", default_name);
  }
  info.type = getTopicType(info.name, yml::readIf(node, "topic_type", default_type));
  info.qos  = parseQoS(node["qos"]);
  return info;
}
}  // namespace stepit
