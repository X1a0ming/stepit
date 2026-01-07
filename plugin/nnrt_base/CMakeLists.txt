cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_nnrt_base)

stepit_add_plugin(${PROJECT_NAME}
    SOURCES src/nnrt.cpp
    INCLUDES include
)

stepit_add_executable(nn-test app/nnrt_test.cpp)
target_link_libraries(nn-test PRIVATE ${PROJECT_NAME})
