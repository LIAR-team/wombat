
wombat_add_library(math STATIC)

target_sources(
  math
  PRIVATE
    src/grid/coordinates.cpp
    src/grid/neighbors.cpp
    src/grid/raytrace.cpp
    src/angles.cpp
    src/online_statistics.cpp
    src/statistics.cpp
    src/transformations.cpp
)

target_include_directories(
  math
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(
  math
  PUBLIC
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
  PRIVATE
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs)

wombat_install_target(math)
