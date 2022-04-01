
add_library(geometry INTERFACE)
add_library(wombat_ops::geometry ALIAS geometry)

target_include_directories(
  geometry
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(geometry INTERFACE geometry_msgs)

wombat_install_target(geometry)
