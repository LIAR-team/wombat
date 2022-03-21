
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
    wombat_control::data_types
)

wombat_install_target(control)
