cmake_minimum_required(VERSION 3.5)
project(wombat_control)

find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(wombat_cmake REQUIRED)
find_package(wombat_core REQUIRED)

wombat_package()

add_subdirectory(control)
add_subdirectory(models)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

wombat_package_install(
  EXPORT_DEPS
    Eigen3
    geometry_msgs
    rclcpp
)
