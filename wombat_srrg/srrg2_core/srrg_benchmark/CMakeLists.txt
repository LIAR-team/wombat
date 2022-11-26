add_library(srrg_benchmark INTERFACE)
add_library(wombat_srrg::srrg2_core::benchmark ALIAS srrg_benchmark)

target_include_directories(
  srrg_benchmark
  INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)

target_link_libraries(
  srrg_benchmark
  INTERFACE
    wombat_srrg::srrg2_core::config
    wombat_srrg::srrg2_core::geometry
    wombat_srrg::srrg2_core::srrg_messages::messages
    wombat_srrg::srrg2_core::system_utils
)

wombat_install_target(srrg_benchmark)
