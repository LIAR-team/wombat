cmake_minimum_required(VERSION 3.5)
project(wombat_strategy)

find_package(ament_cmake REQUIRED)
find_package(wombat_cmake REQUIRED)

find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(wombat_ops REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(wombat_msgs REQUIRED)

wombat_package()

add_subdirectory(exploration)

if(BUILD_TESTING)
  wombat_linters()
endif()

wombat_install_export_targets()

ament_export_dependencies(
  geometry_msgs
  nav_msgs
  nav2_costmap_2d
  nav2_util
  rclcpp
  tf2_eigen
  tf2_ros
  visualization_msgs
  wombat_msgs
  wombat_ops
)

wombat_package_install()
