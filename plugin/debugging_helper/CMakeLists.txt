cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_debugging_helper)

stepit_add_plugin(${PROJECT_NAME}
    SOURCES src/dummy_control.cpp src/dummy_robot.cpp src/exit_spin.cpp
    ENTRY src/entry.cpp
    INCLUDES include
    FLAGS -Wall -Wextra -Wpedantic -Wno-unused-parameter
)
