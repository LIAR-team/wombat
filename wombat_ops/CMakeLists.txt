cmake_minimum_required(VERSION 3.5)
project(wombat_ops)

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

wombat_package_install(
  EXPORT_DEPS
  geometry_msgs nav2_costmap_2d
)
