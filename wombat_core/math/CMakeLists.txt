
add_library(math INTERFACE)
add_library(wombat_core::math ALIAS math)

target_include_directories(
  math
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(math INTERFACE geometry_msgs)

wombat_install_target(math)
