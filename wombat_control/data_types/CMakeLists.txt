
add_library(data_types INTERFACE)
add_library(wombat_control::data_types ALIAS data_types)

target_include_directories(
  data_types
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

wombat_install_target(data_types)
