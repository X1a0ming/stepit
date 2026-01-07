cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_joystick_base)

stepit_add_plugin(${PROJECT_NAME}
    SOURCES src/joystick.cpp src/keymap.cpp
    INCLUDES include
)
