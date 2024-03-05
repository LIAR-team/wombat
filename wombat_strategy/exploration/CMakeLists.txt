
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
    rclcpp::rclcpp
    tf2::tf2
    tf2_eigen::tf2_eigen
    tf2_geometry_msgs::tf2_geometry_msgs
    tf2_ros::tf2_ros
    wombat_core::grid
    wombat_core::math
    wombat_core::ros2
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${wombat_msgs_TARGETS}
  PRIVATE
    wombat_core::costmap
)
ament_target_dependencies(exploration
  PUBLIC
    nav2_util
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
