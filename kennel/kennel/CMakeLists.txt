add_library(kennel_lib SHARED)
target_sources(
  kennel_lib
  PRIVATE
    src/mobile_base/diff_kinematic_model.cpp
    src/mobile_base/ground_truth_manager.cpp
    src/mobile_base/mobile_base.cpp
    src/mobile_base/slam_manager.cpp
    src/kennel.cpp
    src/robot_sim.cpp
)
target_include_directories(
  kennel_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(kennel_lib PUBLIC nav2_map_server tf2 tf2_geometry_msgs tf2_ros)
target_link_libraries(
  kennel_lib
  PUBLIC
    kennel::common
    kennel_sensor_plugins
    rclcpp::rclcpp
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${rosgraph_msgs_TARGETS}
  PRIVATE
    wombat_core::math
)

add_executable(kennel main.cpp)
target_link_libraries(kennel PRIVATE kennel_lib)

wombat_install_target(kennel_lib)
wombat_install_target(kennel)
