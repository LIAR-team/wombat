
add_library(models SHARED)
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
  PRIVATE
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
)
ament_target_dependencies(models PUBLIC geometry_msgs)

wombat_install_target(models)
