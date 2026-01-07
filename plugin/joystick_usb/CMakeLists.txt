cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_joystick_usb)

set(GAMEPAD_ENABLE_SHARED OFF)
set(GAMEPAD_ENABLE_STATIC OFF)
set(GAMEPAD_ENABLE_INSTALL OFF)
set(GAMEPAD_ENABLE_TESTS OFF)
init_submodule(extern/joystick_sdk/libgamepad WORKING_DIRECTORY ${STEPIT_HOME_DIRECTORY})
add_subdirectory(${STEPIT_HOME_DIRECTORY}/extern/joystick_sdk/libgamepad libgamepad)

stepit_add_plugin(${PROJECT_NAME}
    SOURCES src/usb.cpp
    INCLUDES include
    LINK_LIBS gamepad
    DEPENDS joystick_base
)
