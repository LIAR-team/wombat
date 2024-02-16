wombat_add_library(kennel_lib SHARED)
target_sources(
  kennel_lib
  PRIVATE
    src/mobile_base/ground_truth_manager.cpp
    src/mobile_base/mobile_base.cpp
    src/kennel.cpp
    src/sim_time_manager.cpp
    src/robot_sim.cpp
)
target_include_directories(
  kennel_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(kennel_lib PUBLIC nav2_map_server nav2_costmap_2d pluginlib tf2 tf2_geometry_msgs tf2_ros)
target_link_libraries(
  kennel_lib
  PUBLIC
    kennel::common
    rclcpp::rclcpp
    wombat_core::costmap
    wombat_core::ros2
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${nav2_msgs_TARGETS}
    ${rosgraph_msgs_TARGETS}
  PRIVATE
    wombat_core::cpp_utils
    wombat_control::models
    wombat_core::math
)

add_executable(kennel main.cpp)
target_link_libraries(kennel PRIVATE kennel::kennel_lib)

wombat_install_target(kennel_lib)
wombat_install_target(kennel)
