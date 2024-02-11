
wombat_add_library(cpp_utils INTERFACE)

target_include_directories(
  cpp_utils
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

wombat_install_target(cpp_utils)
