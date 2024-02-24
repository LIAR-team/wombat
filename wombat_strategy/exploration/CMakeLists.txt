
wombat_add_library(exploration SHARED)
target_sources(exploration
  PRIVATE
    src/frontier_detector.cpp
    src/frontier_exploration_node.cpp
)
target_include_directories(
  exploration
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(
  exploration
  PUBLIC
  wombat_core::costmap
  wombat_core::grid
  wombat_core::math
)
ament_target_dependencies(exploration
  PUBLIC
    geometry_msgs
    nav2_costmap_2d
    nav_msgs
    nav2_util
    rclcpp
    tf2_eigen
    tf2_ros
    visualization_msgs
    wombat_msgs
)

add_executable(exploration_server src/main.cpp)
target_link_libraries(exploration_server PRIVATE wombat_strategy::exploration)

wombat_install_target(exploration)
wombat_install_target(exploration_server)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
