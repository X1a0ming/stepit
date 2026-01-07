cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_joystick_udp)

set(UDP_GAMEPAD_BUILD_EXAMPLE OFF)
init_submodule(extern/joystick_sdk/udp-gamepad WORKING_DIRECTORY ${STEPIT_HOME_DIRECTORY})
add_subdirectory(${STEPIT_HOME_DIRECTORY}/extern/joystick_sdk/udp-gamepad udp-gamepad)

stepit_add_plugin(${PROJECT_NAME}
    SOURCES src/joystick_udp.cpp
    LINK_LIBS PRIVATE udp_gamepad
    DEPENDS joystick_base
)

install(TARGETS udp_gamepad
    RUNTIME DESTINATION ${STEPIT_BIN_DESTINATION}
    LIBRARY DESTINATION ${STEPIT_LIB_DESTINATION}
    ARCHIVE DESTINATION ${STEPIT_LIB_DESTINATION}
)
