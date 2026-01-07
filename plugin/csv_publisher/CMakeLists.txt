cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_csv_publisher)

stepit_add_plugin(${PROJECT_NAME}
    SOURCES src/csv_publisher.cpp
    INCLUDES include
    FLAGS -Wall -Wextra -Wpedantic -Wno-unused-parameter
)
