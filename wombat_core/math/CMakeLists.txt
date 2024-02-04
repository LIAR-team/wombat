
add_library(math STATIC)
add_library(wombat_core::math ALIAS math)

target_sources(
  math
  PRIVATE
    src/angles.cpp
    src/grid.cpp
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
