
wombat_add_library(costmap STATIC)

target_sources(
  costmap
  PRIVATE
    src/costmap_conversions.cpp
    src/costmap_utils.cpp
    src/inflatable_costmap.cpp
    src/inflation_layer.cpp
    src/static_layer.cpp
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
    ${map_msgs_TARGETS}
    ${nav_msgs_TARGETS}
)
ament_target_dependencies(costmap PUBLIC nav2_costmap_2d tf2)

wombat_install_target(costmap)
