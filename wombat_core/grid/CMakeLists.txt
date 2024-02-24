
wombat_add_library(grid STATIC)

target_sources(
  grid
  PRIVATE
    src/coordinates.cpp
    src/line_iterator.cpp
    src/neighbors.cpp
    src/polygon_iterator.cpp
    src/raytrace.cpp
    src/submap_iterator.cpp
)

target_include_directories(
  grid
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(
  grid
  PUBLIC
    Eigen3::Eigen
    wombat_core::math
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
  PRIVATE
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs)

wombat_install_target(grid)
