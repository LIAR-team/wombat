
add_library(ros2 INTERFACE)
add_library(wombat_core::ros2 ALIAS ros2)

target_include_directories(
  ros2
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(ros2 INTERFACE rclcpp::rclcpp)

wombat_install_target(ros2)
