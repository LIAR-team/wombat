add_library(kennel_common STATIC)
add_library(kennel::common ALIAS kennel_common)
target_sources(
  kennel_common
  PRIVATE
    src/sensors/lidar.cpp
)
target_include_directories(
  kennel_common
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(
  kennel_common
  PUBLIC
    rclcpp::rclcpp
    wombat_core::math
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${rcl_interfaces_TARGETS}
)

wombat_install_target(kennel_common)
