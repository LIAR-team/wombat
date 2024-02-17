wombat_add_library(common STATIC)
target_sources(
  common
  PRIVATE
    src/plugin_interface/map_positioner.cpp
    src/plugin_interface/plugin_base.cpp
    src/sensors/lidar.cpp
)
target_include_directories(
  common
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(
  common
  PUBLIC
    rclcpp::rclcpp
    wombat_core::math
    wombat_core::ros2
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${rcl_interfaces_TARGETS}
)

wombat_install_target(common)
