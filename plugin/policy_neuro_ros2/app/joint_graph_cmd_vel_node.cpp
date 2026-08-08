#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <stepit/nnrt/onnxruntime.h>

namespace {

constexpr std::size_t kObservationSize = 458;
constexpr std::size_t kActionSize = 32;
constexpr std::size_t kPathSize = 40;
constexpr std::size_t kParentOffset = 371;
constexpr std::size_t kMaxJoints = 29;

bool isFiniteValue(double value) {
  return std::isfinite(value);
}

std::array<float, 3> projectedGravity(const sensor_msgs::msg::Imu &imu) {
  double w = imu.orientation.w;
  double x = imu.orientation.x;
  double y = imu.orientation.y;
  double z = imu.orientation.z;
  const double norm = std::sqrt(w * w + x * x + y * y + z * z);
  if (not isFiniteValue(norm) or norm < 1.0e-8) {
    throw std::runtime_error("IMU orientation quaternion is invalid");
  }
  w /= norm;
  x /= norm;
  y /= norm;
  z /= norm;
  return {
      static_cast<float>(2.0 * (w * y - x * z)),
      static_cast<float>(-2.0 * (y * z + w * x)),
      static_cast<float>(2.0 * (x * x + y * y) - 1.0),
  };
}

bool validPathLayout(const std_msgs::msg::Float32MultiArray &message) {
  if (message.layout.data_offset != 0 or message.layout.dim.size() != 2) return false;
  const auto &points = message.layout.dim[0];
  const auto &xy = message.layout.dim[1];
  return points.label == "point" and points.size == 20 and points.stride == 40 and
         xy.label == "xy" and xy.size == 2 and xy.stride == 2;
}

}  // namespace

class JointGraphCmdVelNode final : public rclcpp::Node {
 public:
  JointGraphCmdVelNode() : Node("joint_graph_cmd_vel") {
    model_path_ = declare_parameter<std::string>("model_path", "");
    model_config_path_ = declare_parameter<std::string>("model_config_path", "");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/stepit/imu");
    path_topic_ = declare_parameter<std::string>("path_topic", "/stepit/path_points_body");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/stepit/cmd_vel");
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 50.0);
    imu_timeout_s_ = declare_parameter<double>("imu_timeout_s", 0.10);
    path_timeout_s_ = declare_parameter<double>("path_timeout_s", 0.25);
    ang_vel_scale_ = declare_parameter<double>("ang_vel_scale", 0.25);
    path_scale_ = declare_parameter<double>("path_scale", 1.0);
    previous_cmd_scale_ = declare_parameter<double>("previous_cmd_scale", 1.0);
    robot_id_ = declare_parameter<int>("robot_id", 8);
    require_path_layout_ = declare_parameter<bool>("require_path_layout", true);
    const auto lower = declare_parameter<std::vector<double>>(
        "cmd_vel_lower", std::vector<double>{-2.0, -2.0, -2.0});
    const auto upper = declare_parameter<std::vector<double>>(
        "cmd_vel_upper", std::vector<double>{2.0, 2.0, 2.0});

    if (model_path_.empty() or model_config_path_.empty()) {
      throw std::invalid_argument("model_path and model_config_path are required");
    }
    if (not isFiniteValue(control_rate_hz_) or control_rate_hz_ <= 0.0 or
        not isFiniteValue(imu_timeout_s_) or imu_timeout_s_ <= 0.0 or
        not isFiniteValue(path_timeout_s_) or path_timeout_s_ <= 0.0 or
        not isFiniteValue(ang_vel_scale_) or not isFiniteValue(path_scale_) or
        not isFiniteValue(previous_cmd_scale_)) {
      throw std::invalid_argument("rates, timeouts and observation scales must be finite and positive");
    }
    if (robot_id_ < 0 or robot_id_ > 8) {
      throw std::invalid_argument("robot_id must be in [0, 8]");
    }
    if (lower.size() != 3 or upper.size() != 3) {
      throw std::invalid_argument("cmd_vel_lower and cmd_vel_upper must each contain three values");
    }
    for (std::size_t index = 0; index < 3; ++index) {
      if (not isFiniteValue(lower[index]) or not isFiniteValue(upper[index]) or
          lower[index] > upper[index]) {
        throw std::invalid_argument("cmd_vel limits must be finite and lower <= upper");
      }
      cmd_lower_[index] = static_cast<float>(lower[index]);
      cmd_upper_[index] = static_cast<float>(upper[index]);
    }

    model_ = std::make_unique<stepit::OnnxRt>(model_path_, llu::yml::loadFile(model_config_path_));
    if (model_->getInputSize("observation") != kObservationSize or
        model_->getOutputSize("action") != kActionSize) {
      throw std::runtime_error("Joint-graph actor must have observation[458] and action[32]");
    }
    model_->clearState();
    model_->warmup(3);
    model_->clearState();

    observation_.fill(0.0F);
    std::fill_n(observation_.begin() + kParentOffset, kMaxJoints, -1.0F);
    observation_[49] = static_cast<float>(robot_id_);
    observation_[50] = 1.0F;
    observation_[51] = 0.0F;

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    imu_subscriber_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, sensor_qos,
        [this](sensor_msgs::msg::Imu::SharedPtr message) { receiveImu(std::move(message)); });
    path_subscriber_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        path_topic_, sensor_qos,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr message) { receivePath(std::move(message)); });
    cmd_publisher_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, sensor_qos);

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                               [this]() { update(); });
    publishZero();
    RCLCPP_INFO(get_logger(),
                "V12 joint-graph cmd_vel node ready: model=%s, imu=%s, path=%s, cmd=%s, rate=%.1f Hz",
                model_path_.c_str(), imu_topic_.c_str(), path_topic_.c_str(), cmd_vel_topic_.c_str(),
                control_rate_hz_);
  }

  ~JointGraphCmdVelNode() override {
    publishZero();
  }

 private:
  using Clock = std::chrono::steady_clock;

  void receiveImu(const sensor_msgs::msg::Imu::SharedPtr message) {
    const auto &angular = message->angular_velocity;
    const auto &orientation = message->orientation;
    if (not isFiniteValue(angular.x) or not isFiniteValue(angular.y) or
        not isFiniteValue(angular.z) or not isFiniteValue(orientation.w) or
        not isFiniteValue(orientation.x) or not isFiniteValue(orientation.y) or
        not isFiniteValue(orientation.z)) {
      std::lock_guard<std::mutex> lock(mutex_);
      imu_valid_ = false;
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    imu_ = *message;
    imu_time_ = Clock::now();
    imu_valid_ = true;
  }

  void receivePath(const std_msgs::msg::Float32MultiArray::SharedPtr message) {
    bool valid = message->data.size() == kPathSize and
                 (not require_path_layout_ or validPathLayout(*message));
    for (const float value : message->data) valid = valid and std::isfinite(value);
    std::lock_guard<std::mutex> lock(mutex_);
    if (not valid) {
      path_valid_ = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Rejected path: expected 20x2 finite point_xy Float32MultiArray");
      return;
    }
    std::copy_n(message->data.begin(), kPathSize, path_.begin());
    path_time_ = Clock::now();
    path_valid_ = true;
  }

  void resetInferenceState() {
    if (inference_active_) model_->clearState();
    inference_active_ = false;
    previous_cmd_.fill(0.0F);
  }

  void publishZero() {
    if (not cmd_publisher_) return;
    cmd_publisher_->publish(geometry_msgs::msg::Twist{});
  }

  void update() {
    sensor_msgs::msg::Imu imu;
    std::array<float, kPathSize> path{};
    bool ready = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto now = Clock::now();
      ready = imu_valid_ and path_valid_ and
              std::chrono::duration<double>(now - imu_time_).count() <= imu_timeout_s_ and
              std::chrono::duration<double>(now - path_time_).count() <= path_timeout_s_;
      if (ready) {
        imu = imu_;
        path = path_;
      }
    }

    if (not ready) {
      if (inference_active_) {
        RCLCPP_WARN(get_logger(), "IMU/path stream became stale; publishing zero cmd_vel and clearing state");
      }
      resetInferenceState();
      publishZero();
      return;
    }

    try {
      const auto gravity = projectedGravity(imu);
      observation_[0] = static_cast<float>(imu.angular_velocity.x * ang_vel_scale_);
      observation_[1] = static_cast<float>(imu.angular_velocity.y * ang_vel_scale_);
      observation_[2] = static_cast<float>(imu.angular_velocity.z * ang_vel_scale_);
      std::copy(gravity.begin(), gravity.end(), observation_.begin() + 3);
      for (std::size_t index = 0; index < kPathSize; ++index) {
        observation_[6 + index] = static_cast<float>(path[index] * path_scale_);
      }
      for (std::size_t index = 0; index < 3; ++index) {
        observation_[46 + index] = static_cast<float>(previous_cmd_[index] * previous_cmd_scale_);
      }

      model_->setInput("observation", observation_.data());
      model_->runInference();
      const float *output = model_->getOutput<float>("action");
      geometry_msgs::msg::Twist message;
      for (std::size_t index = 0; index < 3; ++index) {
        if (not std::isfinite(output[index])) throw std::runtime_error("actor produced non-finite cmd_vel");
        previous_cmd_[index] = std::clamp(output[index], cmd_lower_[index], cmd_upper_[index]);
      }
      message.linear.x = previous_cmd_[0];
      message.linear.y = previous_cmd_[1];
      message.angular.z = previous_cmd_[2];
      cmd_publisher_->publish(message);
      inference_active_ = true;
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "Joint-graph inference failed: %s", error.what());
      resetInferenceState();
      publishZero();
    }
  }

  std::string model_path_;
  std::string model_config_path_;
  std::string imu_topic_;
  std::string path_topic_;
  std::string cmd_vel_topic_;
  double control_rate_hz_{};
  double imu_timeout_s_{};
  double path_timeout_s_{};
  double ang_vel_scale_{};
  double path_scale_{};
  double previous_cmd_scale_{};
  int robot_id_{};
  bool require_path_layout_{};

  std::unique_ptr<stepit::OnnxRt> model_;
  std::array<float, kObservationSize> observation_{};
  std::array<float, kPathSize> path_{};
  std::array<float, 3> previous_cmd_{};
  std::array<float, 3> cmd_lower_{};
  std::array<float, 3> cmd_upper_{};
  sensor_msgs::msg::Imu imu_;
  Clock::time_point imu_time_{};
  Clock::time_point path_time_{};
  bool imu_valid_{};
  bool path_valid_{};
  bool inference_active_{};
  std::mutex mutex_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr path_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<JointGraphCmdVelNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("joint_graph_cmd_vel"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
