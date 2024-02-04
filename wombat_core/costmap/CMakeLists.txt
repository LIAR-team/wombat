
add_library(costmap STATIC)
add_library(wombat_core::costmap ALIAS costmap)

target_sources(
  costmap
  PRIVATE
    src/costmap.cpp
)

target_include_directories(
  costmap
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(
  costmap
  PUBLIC
    ${geometry_msgs_TARGETS}
)
ament_target_dependencies(costmap PUBLIC nav2_costmap_2d)

wombat_install_target(costmap)
