
add_library(kennel_sensor_plugins SHARED)
target_sources(
  kennel_sensor_plugins
  PRIVATE
    src/lidar_2d.cpp
)
target_include_directories(
  kennel_sensor_plugins
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(
  kennel_sensor_plugins
  PRIVATE
    kennel::common
    ${sensor_msgs_TARGETS}
)

wombat_install_target(kennel_sensor_plugins)
