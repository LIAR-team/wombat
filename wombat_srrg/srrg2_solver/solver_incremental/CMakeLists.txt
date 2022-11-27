###########
## solver_incremental library
############
add_library(srrg2_solver_solver_incremental SHARED)
add_library(wombat_srrg::srrg2_solver::solver_incremental ALIAS srrg2_solver_solver_incremental)
target_sources(
  srrg2_solver_solver_incremental
  PRIVATE
    src/instances.cpp

    src/factor_graph_incremental_sorter.cpp
    src/solver_incremental_base.cpp
    src/solver_incremental.cpp
    src/solver_incremental_running_environment.cpp
    src/solver_incremental_runner.cpp
)
target_include_directories(
  srrg2_solver_solver_incremental
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(
  srrg2_solver_solver_incremental
  PUBLIC
    wombat_srrg::srrg2_core::boss
    wombat_srrg::srrg2_core::config
    wombat_srrg::srrg2_solver::core
    wombat_srrg::srrg2_solver::types_2d
    wombat_srrg::srrg2_solver::types_3d
    wombat_srrg::srrg2_solver::types_projective
    wombat_srrg::srrg2_solver::types_calib
    wombat_srrg::srrg2_solver::utils
)

wombat_install_target(srrg2_solver_solver_incremental)
