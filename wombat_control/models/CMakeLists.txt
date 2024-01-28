
add_library(models STATIC)
add_library(wombat_control::models ALIAS models)

target_sources(
  models
  PRIVATE
    src/diff_drive_model.cpp
)

target_include_directories(
  models
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(
  models
  PUBLIC
    rclcpp::rclcpp
    ${geometry_msgs_TARGETS}
  PRIVATE
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    wombat_core::math
)

wombat_install_target(models)
