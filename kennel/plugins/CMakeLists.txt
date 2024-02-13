
wombat_add_library(kennel_sensor_plugins SHARED)
target_sources(
  kennel_sensor_plugins
  PRIVATE
    src/bumper.cpp
    src/lidar_2d.cpp
)
target_link_libraries(
  kennel_sensor_plugins
  PRIVATE
    kennel::common
    ${sensor_msgs_TARGETS}
    ${wombat_msgs_TARGETS}
)
ament_target_dependencies(kennel_sensor_plugins PUBLIC pluginlib)

wombat_install_target(kennel_sensor_plugins)
