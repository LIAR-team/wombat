wombat_add_library(common STATIC)
target_sources(
  common
  PRIVATE
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
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${rcl_interfaces_TARGETS}
)

wombat_install_target(common)
