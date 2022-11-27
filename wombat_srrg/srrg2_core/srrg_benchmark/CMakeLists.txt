add_library(srrg_benchmark SHARED)
add_library(wombat_srrg::srrg2_core::benchmark ALIAS srrg_benchmark)

target_include_directories(
  srrg_benchmark
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)

target_sources(
  srrg_benchmark
  PRIVATE
    src/trajectory_writers.cpp
)

target_link_libraries(
  srrg_benchmark
  PUBLIC
    wombat_srrg::srrg2_core::config
    wombat_srrg::srrg2_core::geometry
    wombat_srrg::srrg2_core::srrg_messages::messages
    wombat_srrg::srrg2_core::system_utils
)

wombat_install_target(srrg_benchmark)
