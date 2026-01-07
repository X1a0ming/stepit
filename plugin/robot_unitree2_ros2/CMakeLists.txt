cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_robot_unitree2_ros2)

# find dependencies
find_package(ament_cmake QUIET)
if (NOT ament_cmake_FOUND)
  message(STATUS "Skipped plugin '${PROJECT_NAME}': Missing ament_cmake.")
  return()
endif ()
find_package(unitree_go QUIET)
if (NOT unitree_go_FOUND)
  message(STATUS "Skipped plugin '${PROJECT_NAME}': Missing unitree_go.")
  return()
endif ()

stepit_add_plugin(${PROJECT_NAME}
    SOURCES src/joystick.cpp src/quadruped.cpp
    INCLUDES include
    DEPENDS robot_unitree2 ros2_base
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC unitree_go)


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
