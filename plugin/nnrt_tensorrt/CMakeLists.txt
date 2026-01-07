cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_nnrt_tensorrt)

find_package(CUDA QUIET)
if (NOT CUDA_FOUND)
  message(STATUS "Skipped plugin '${PROJECT_NAME}': Missing CUDA.")
  return()
endif ()
get_library_directory(nvinfer NVINFER_DIR)
if (NOT NVINFER_DIR)
  message(STATUS "Skipped plugin '${PROJECT_NAME}': Missing TensorRT.")
  return()
endif ()

stepit_add_plugin(${PROJECT_NAME}
    SOURCES src/tensorrt.cpp
    INCLUDES include ${CUDA_INCLUDE_DIRS}
    LINK_DIRS ${NVINFER_DIR}
    LINK_LIBS ${CUDA_LIBRARIES} nvinfer nvonnxparser
    DEPENDS nnrt_base
)
