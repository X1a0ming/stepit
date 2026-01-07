#ifndef STEPIT_ROS2_PUBLISHER_H_
#define STEPIT_ROS2_PUBLISHER_H_

#include <map>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <stepit/publisher.h>

namespace stepit {
class Ros2Publisher : public Publisher {
 public:
  Ros2Publisher();
  void publishStatus() override;
  void publishLowLevel(const RobotSpec &spec, const LowState &state, const LowCmd &cmd) override;
  void publishArray(const std::string &name, cArrXf arr) override;

 private:
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr> pub_map_;

  diagnostic_msgs::msg::DiagnosticStatus status_msg_;
  sensor_msgs::msg::JointState joint_msg_;
  sensor_msgs::msg::Imu imu_msg_;
  geometry_msgs::msg::Twist twist_msg_;
};
}  // namespace stepit

#endif  // STEPIT_ROS2_PUBLISHER_H_
