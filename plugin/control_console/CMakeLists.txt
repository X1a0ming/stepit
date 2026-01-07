cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_control_console)

stepit_add_plugin(${PROJECT_NAME}
    SOURCES src/control_console.cpp
    INCLUDES include
)
