cmake_minimum_required(VERSION 3.5)
project(kennel)

find_package(wombat_cmake REQUIRED)
set(PKG_DEPENDENCIES
  geometry_msgs
  nav_msgs
  nav2_costmap_2d
  nav2_map_server
  pluginlib
  rclcpp
  rcl_interfaces
  rosgraph_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  wombat_control
  wombat_core
  wombat_msgs
)
foreach(PKG ${PKG_DEPENDENCIES})
  find_package(${PKG} REQUIRED)
endforeach()

wombat_package()

add_subdirectory(bringup)
add_subdirectory(common)
add_subdirectory(kennel)
add_subdirectory(plugins)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

pluginlib_export_plugin_description_file(kennel plugins/sensor_plugins.xml)
wombat_package_install(
  EXPORT_DEPS
    ${PKG_DEPENDENCIES}
)
