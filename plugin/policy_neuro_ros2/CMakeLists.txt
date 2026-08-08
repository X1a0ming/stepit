cmake_minimum_required(VERSION 3.23)
project(stepit_plugin_policy_neuro_ros2)
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(grid_map_msgs QUIET)
find_package(grid_map_ros QUIET)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

stepit_add_plugin(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PRIVATE
      src/cmd_posture_subscriber2.cpp
      src/cmd_vel_subscriber2.cpp
      src/field_subscriber2.cpp
    PUBLIC
      FILE_SET HEADERS
      BASE_DIRS include
      FILES
        include/stepit/policy_neuro_ros2/cmd_posture_subscriber2.h
        include/stepit/policy_neuro_ros2/cmd_vel_subscriber2.h
        include/stepit/policy_neuro_ros2/field_subscriber2.h
)
if (grid_map_msgs_FOUND AND grid_map_ros_FOUND)
  target_sources(${PROJECT_NAME}
      PRIVATE src/heightmap_subscriber2.cpp
      PUBLIC FILE_SET HEADERS FILES
        include/stepit/policy_neuro_ros2/heightmap_subscriber2.h
  )
  ament_target_dependencies(${PROJECT_NAME} PUBLIC grid_map_msgs grid_map_ros)
else ()
  message(STATUS "policy_neuro_ros2: grid_map packages not found; heightmap_subscriber2 disabled")
endif ()
target_link_libraries(${PROJECT_NAME} PUBLIC
    stepit_core
    stepit_plugin_policy_neuro
    stepit_plugin_ros2_base
)
install(TARGETS ${PROJECT_NAME}
    EXPORT stepitTargets
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
    FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
    geometry_msgs
    nav_msgs
    rclcpp
    sensor_msgs
    std_msgs
    tf2_ros
)

add_executable(joint_graph_cmd_vel_node app/joint_graph_cmd_vel_node.cpp)
target_link_libraries(joint_graph_cmd_vel_node
    stepit_plugin_nnrt_onnxruntime
)
ament_target_dependencies(joint_graph_cmd_vel_node
    geometry_msgs
    rclcpp
    sensor_msgs
    std_msgs
)
install(TARGETS joint_graph_cmd_vel_node
    RUNTIME DESTINATION ${STEPIT_BIN_DESTINATION}
)


if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
