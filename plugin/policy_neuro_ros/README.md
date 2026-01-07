# policy_neuro_ros

StepIt plugin for ROS-based field sources that subscribe ROS topics and feed data into the StepIt neuro policy.

Provided factories:

`stepit::policy_neuro::FieldSource`:
  - `cmd_vel`: subscribes to a `geometry_msgs/Twist` or `geometry_msgs/TwistStamped` topic and provides command velocity fields (`cmd_vel`) with `linear.x`, `linear.y`, and `angular.z` components.
  - `cmd_pitch`: subscribes to a `std_msgs/Float32`, `geometry_msgs/Twist` or `geometry_msgs/TwistStamped` topic and provides a command pitch field (`cmd_pitch`). If the topic is of type `geometry_msgs/Twist` or `geometry_msgs/TwistStamped`, the pitch command is taken from the `angular.y` component.
  - `cmd_height`: subscribes to a `std_msgs/Float32`, `geometry_msgs/Twist` or `geometry_msgs/TwistStamped` topic and provides a command height field (`cmd_height`). If the topic is of type `geometry_msgs/Twist` or `geometry_msgs/TwistStamped`, the height command is taken from the `linear.z` component.
  - `heightmap` / `heightmap_uncertainty` : subscribe to a `grid_map_msgs/GridMap` and a pose topic (`geometry_msgs/Pose`, `geometry_msgs/PoseStamped`, `geometry_msgs/PoseWithCovariance`, `geometry_msgs/PoseWithCovarianceStamped`, or `nav_msgs/Odometry`), sample elevation and uncertainty values around the robot, and provide corresponding fields (`heightmap` / `heightmap_uncertainty`).
