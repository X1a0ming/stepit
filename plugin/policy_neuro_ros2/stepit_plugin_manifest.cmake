stepit_declare_plugin(NAME policy_neuro_ros2 DEPENDS
    nnrt_onnxruntime
    policy_neuro
    ros2_base
)
set(missing_packages "")
find_package(nav_msgs QUIET)
if (NOT nav_msgs_FOUND)
  list(APPEND missing_packages nav_msgs)
endif ()
find_package(tf2_ros QUIET)
if (NOT tf2_ros_FOUND)
  list(APPEND missing_packages tf2_ros)
endif ()
if (missing_packages)
  string(REPLACE ";" ", " missing_packages_text "${missing_packages}")
  stepit_plugin_mark_unbuildable("Missing CMake package(s): ${missing_packages_text}.")
  return()
endif ()
