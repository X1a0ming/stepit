cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_robot_deeprobotics_x30)

init_submodule(extern/robot_sdk/deeprobotics_x30_sdk WORKING_DIRECTORY ${STEPIT_HOME_DIRECTORY})
set(X30_SDK_HOME ${STEPIT_HOME_DIRECTORY}/extern/robot_sdk/deeprobotics_x30_sdk)

stepit_add_plugin(${PROJECT_NAME}
    SOURCES
      ${X30_SDK_HOME}/src/command.cpp
      ${X30_SDK_HOME}/src/command_list.cpp
      ${X30_SDK_HOME}/src/parse_cmd.cpp
      ${X30_SDK_HOME}/src/send_to_robot.cpp
      src/deeprobotics_x30.cpp
    INCLUDES
      include
      ${X30_SDK_HOME}/include
)
