# policy_neuro_ros2

StepIt plugin for ROS2-based field sources that subscribe ROS2 topics and feed data into the StepIt neuro policy.

Provided factories:

`stepit::policy_neuro::FieldSource`:

- `cmd_height`: subscribes to `std_msgs/msg/Float32`, `geometry_msgs/msg/Twist`, or `geometry_msgs/msg/TwistStamped` on a ROS2 topic and provides command height field (`cmd_height`).
- `cmd_pitch`: subscribes to `std_msgs/msg/Float32`, `geometry_msgs/msg/Twist`, or `geometry_msgs/msg/TwistStamped` on a ROS2 topic and provides command pitch field (`cmd_pitch`).
- `cmd_roll`: subscribes to `std_msgs/msg/Float32`, `geometry_msgs/msg/Twist`, or `geometry_msgs/msg/TwistStamped` on a ROS2 topic and provides command roll field (`cmd_roll`).
- `cmd_vel`: subscribes to `geometry_msgs/msg/Twist` or `geometry_msgs/msg/TwistStamped` on a ROS2 topic and provides command velocity fields (`cmd_vel`).
- `heightmap` / `heightmap_uncertainty`: subscribe to a `grid_map_msgs/msg/GridMap` and a pose topic (`geometry_msgs/msg/Pose`, `geometry_msgs/msg/PoseStamped`, `geometry_msgs/msg/PoseWithCovariance`, `geometry_msgs/msg/PoseWithCovarianceStamped`, or `nav_msgs/msg/Odometry`), sample elevation and uncertainty values around the robot, and provide corresponding fields (`heightmap` / `heightmap_uncertainty`).
