add_library(srrg_config SHARED)
add_library(wombat_srrg::srrg2_core::srrg_config ALIAS srrg_config)

target_sources(
  srrg_config
  PRIVATE
    src/configurable.cpp
    src/preemptible.cpp
)

target_include_directories(
  srrg_config
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)

target_link_libraries(
  srrg_config
  PUBLIC
    wombat_srrg::srrg2_core::srrg_boss
    wombat_srrg::srrg2_core::srrg_data_structures
    wombat_srrg::srrg2_core::srrg_property
    wombat_srrg::srrg2_core::srrg_system_utils
)

wombat_install_target(srrg_config)
