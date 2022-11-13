add_library(srrg_path_matrix SHARED)
add_library(wombat_srrg::srrg2_core::srrg_path_matrix ALIAS srrg_path_matrix)

target_sources(
  srrg_path_matrix
  PRIVATE
    src/path_matrix_astar_search.cpp
    src/path_matrix_clusterer_search.cpp
    src/path_matrix_cost_search.cpp
    src/path_matrix_dijkstra_search.cpp
    src/path_matrix_distance_search.cpp
    src/path_matrix.cpp
)

target_include_directories(
  srrg_path_matrix
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)

target_link_libraries(
  srrg_path_matrix
  PUBLIC
    wombat_srrg::srrg2_core::srrg_geometry
    wombat_srrg::srrg2_core::srrg_image
    wombat_srrg::srrg2_core::property
    wombat_srrg::srrg2_core::pcl
    wombat_srrg::srrg2_core::system_utils
)

wombat_install_target(srrg_path_matrix)
