#ifndef STEPIT_ROS_PUBLISHER_H_
#define STEPIT_ROS_PUBLISHER_H_

#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>

#include <stepit/publisher.h>

namespace stepit {
class RosPublisher : public Publisher {
 public:
  RosPublisher();
  void publishStatus() override;
  void publishLowLevel(const RobotSpec &spec, const LowState &state, const LowCmd &cmd) override;
  void publishArray(const std::string &name, cArrXf arr) override;

 private:
  ros::Publisher status_pub_;
  ros::Publisher joint_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher twist_pub_;
  std::map<std::string, ros::Publisher> pub_map_;

  diagnostic_msgs::DiagnosticStatus status_msg_;
  sensor_msgs::JointState joint_msg_;
  sensor_msgs::Imu imu_msg_;
  geometry_msgs::Twist twist_msg_;
};
}  // namespace stepit

#endif  // STEPIT_ROS_PUBLISHER_H_
