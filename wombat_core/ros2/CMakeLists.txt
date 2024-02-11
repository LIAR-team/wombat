
wombat_add_library(ros2 STATIC)

target_sources(
  ros2
  PRIVATE
    src/parameters.cpp
)

target_include_directories(
  ros2
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(ros2 PUBLIC rclcpp::rclcpp)

wombat_install_target(ros2)
