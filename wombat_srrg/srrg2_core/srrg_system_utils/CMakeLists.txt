add_library(srrg_system_utils SHARED)
add_library(wombat_srrg::srrg2_core::system_utils ALIAS srrg_system_utils)

target_sources(
  srrg_system_utils
  PRIVATE
    src/chrono.cpp
    src/env.cpp
    src/system_utils.cpp
    src/parse_command_line.cpp
    src/profiler.cpp
)

target_include_directories(
  srrg_system_utils
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)

target_link_libraries(
  srrg_system_utils
  PUBLIC
    Eigen3::Eigen
)

wombat_install_target(srrg_system_utils)
