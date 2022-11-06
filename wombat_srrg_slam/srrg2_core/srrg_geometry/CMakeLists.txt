add_library(srrg_geometry SHARED)
add_library(wombat_srrg::srrg2_core::srrg_geometry ALIAS srrg_geometry)

target_sources(
  srrg_geometry
  PRIVATE
    src/epipolar.cpp
    src/permutation_sampler.cpp
)

target_include_directories(
  srrg_geometry
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)

target_link_libraries(
  srrg_geometry
  PUBLIC
    Eigen3::Eigen
)

wombat_install_target(srrg_geometry)
