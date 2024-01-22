cmake_minimum_required(VERSION 3.5)
project(wombat_core)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(wombat_cmake REQUIRED)

wombat_package()

add_subdirectory(math)
add_subdirectory(costmap)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

wombat_package_install(
  EXPORT_DEPS
    geometry_msgs
    nav_msgs
    nav2_costmap_2d
    tf2
    tf2_geometry_msgs
)
