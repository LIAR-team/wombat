find_package(ament_cmake_gtest REQUIRED)

wombat_add_library(kennel_gtest STATIC)
target_sources(
  kennel_gtest
  PRIVATE
    src/kennel_config.cpp
    src/single_robot_fixture.cpp
)
target_include_directories(
  kennel_gtest
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(
  kennel_gtest
  PUBLIC
    kennel::common
    kennel::kennel_lib
    rclcpp::rclcpp
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    ${wombat_msgs_TARGETS}
)
wombat_install_target(kennel_gtest)
