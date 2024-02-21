
find_package(Boost REQUIRED)

wombat_add_library(kennel_sensor_plugins SHARED)
target_sources(
  kennel_sensor_plugins
  PRIVATE
    bumper.cpp
    lidar_2d.cpp
    lidar_2dsimplified.cpp
)
target_link_libraries(
  kennel_sensor_plugins
  PRIVATE
    kennel::common
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    ${sensor_msgs_TARGETS}
    ${wombat_msgs_TARGETS}
)
ament_target_dependencies(kennel_sensor_plugins PUBLIC pluginlib)
wombat_install_target(kennel_sensor_plugins)
