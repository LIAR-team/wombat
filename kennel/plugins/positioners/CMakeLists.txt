
find_package(Boost REQUIRED)

wombat_add_library(kennel_positioner_plugins SHARED)
target_sources(
  kennel_positioner_plugins
  PRIVATE
    lidar_slam_positioner.cpp
    local_slam_positioner.cpp
    stub_positioner.cpp
)
target_link_libraries(
  kennel_positioner_plugins
  PRIVATE
    Boost::boost
    kennel::common
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    wombat_core::grid
    wombat_core::math
)
ament_target_dependencies(kennel_positioner_plugins PUBLIC pluginlib)
wombat_install_target(kennel_positioner_plugins)
