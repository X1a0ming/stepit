cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_robot_deeprobotics_lite3)

if (NOT CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|aarch64")
  message(WARNING "Skipped plugin '${PROJECT_NAME}': Requires x86_64 or aarch64 architectures, got '${CMAKE_SYSTEM_PROCESSOR}'.")
endif ()

init_submodule(extern/robot_sdk/deeprobotics_lite3_sdk WORKING_DIRECTORY ${STEPIT_HOME_DIRECTORY})
set(LITE3_SDK_HOME ${STEPIT_HOME_DIRECTORY}/extern/robot_sdk/deeprobotics_lite3_sdk)

stepit_add_plugin(${PROJECT_NAME}
    SOURCES src/deeprobotics_lite3.cpp
    INCLUDES include ${LITE3_SDK_HOME}/include ${LITE3_SDK_HOME}/include/common
    LINK_LIBS deeprobotics_legged_sdk_${CMAKE_SYSTEM_PROCESSOR}
    LINK_DIRS ${LITE3_SDK_HOME}/lib
)
