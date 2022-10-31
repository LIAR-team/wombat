
add_library(control SHARED)
add_library(wombat_control::control ALIAS control)

target_sources(
  control
  PRIVATE
    src/position_controller.cpp
)
target_include_directories(
  control
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(
  control
  PUBLIC
    Eigen3::Eigen
  PRIVATE
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    wombat_core::math
)
ament_target_dependencies(control PUBLIC geometry_msgs)

wombat_install_target(control)
