cmake_minimum_required(VERSION 3.5)
project(wombat_apps)

find_package(ament_cmake REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(nav2_planner REQUIRED)
find_package(nav2_recoveries REQUIRED)
find_package(nav2_smoother REQUIRED)
find_package(nav2_waypoint_follower REQUIRED)
find_package(rclcpp REQUIRED)
find_package(wombat_cmake REQUIRED)

wombat_package()

add_executable(wombat_nav src/wombat_nav.cpp)
ament_target_dependencies(
  wombat_nav
  PUBLIC
    nav2_bt_navigator
    nav2_controller
    nav2_lifecycle_manager
    nav2_planner
    nav2_recoveries
    nav2_smoother
    nav2_waypoint_follower
    rclcpp
)

if(BUILD_TESTING)
  wombat_linters()
endif()

install(TARGETS wombat_nav
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
