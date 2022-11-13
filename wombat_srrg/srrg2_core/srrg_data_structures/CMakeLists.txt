add_library(srrg_data_structures SHARED)
add_library(wombat_srrg::srrg2_core::data_structures ALIAS srrg_data_structures)

target_sources(
  srrg_data_structures
  PRIVATE
    src/graph.cpp
    src/grid_map_2d.cpp
    src/link.cpp
    src/platform.cpp
)

target_include_directories(
  srrg_data_structures
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)

target_link_libraries(
  srrg_data_structures
  PUBLIC
    wombat_srrg::srrg2_core::srrg_geometry
    wombat_srrg::srrg2_core::property
    wombat_srrg::srrg2_core::system_utils
    Eigen3::Eigen
)

wombat_install_target(srrg_data_structures)
