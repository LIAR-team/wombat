
add_library(cpp_utils INTERFACE)
add_library(wombat_core::cpp_utils ALIAS cpp_utils)

target_include_directories(
  cpp_utils
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

wombat_install_target(cpp_utils)
