cmake_minimum_required(VERSION 3.5)
project(wombat_core)

find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(wombat_cmake REQUIRED)

wombat_package()

add_subdirectory(costmap)
add_subdirectory(cpp)
add_subdirectory(grid)
add_subdirectory(math)
add_subdirectory(ros2)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

wombat_package_install(
  EXPORT_DEPS
    Eigen3
    geometry_msgs
    map_msgs
    nav_msgs
    nav2_costmap_2d
    rclcpp
    tf2
    tf2_geometry_msgs
)
