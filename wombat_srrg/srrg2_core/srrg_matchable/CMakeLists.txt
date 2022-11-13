add_library(srrg_matchable SHARED)
add_library(wombat_srrg::srrg2_core::matchable ALIAS srrg_matchable)

target_sources(
  srrg_matchable
  PRIVATE
    src/instances.cpp
)

target_include_directories(
  srrg_matchable
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)

target_link_libraries(
  srrg_matchable
  PUBLIC
    wombat_srrg::srrg2_core::geometry
    wombat_srrg::srrg2_core::pcl
  PRIVATE
    wombat_srrg::srrg2_core::boss
)

wombat_install_target(srrg_matchable)
