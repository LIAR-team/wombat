cmake_minimum_required(VERSION 3.5)
project(wombat_ops)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(wombat_cmake REQUIRED)

wombat_package()

add_subdirectory(costmap)
add_subdirectory(geometry)

if(BUILD_TESTING)
  wombat_linters()
  add_subdirectory(test)
endif()

wombat_install_export_targets()

ament_export_dependencies(geometry_msgs nav2_costmap_2d)
ament_package()
