
###########
## sparse_block_matrix library
############
add_library(srrg2_solver_sparse_block_matrix SHARED)
add_library(wombat_srrg::srrg2_solver::sparse_block_matrix ALIAS srrg2_solver_sparse_block_matrix)
target_sources(
  srrg2_solver_sparse_block_matrix
  PRIVATE
    src/sparse_block_matrix/matrix_block.cpp
    src/sparse_block_matrix/sparse_block_cholesky.cpp
    src/sparse_block_matrix/matrix_block_factory.cpp
    src/sparse_block_matrix/sparse_block_matrix.cpp
)
target_include_directories(
  srrg2_solver_sparse_block_matrix
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(
  srrg2_solver_sparse_block_matrix
  PUBLIC
    Eigen3::Eigen
)

###########
## linear_solvers library
############
add_library(srrg2_solver_linear_solvers SHARED)
add_library(wombat_srrg::srrg2_solver::linear_solvers ALIAS srrg2_solver_linear_solvers)
target_sources(
  srrg2_solver_linear_solvers
  PRIVATE
    src/linear_solvers/sparse_block_linear_solver.cpp
    src/linear_solvers/sparse_block_linear_solver_cholesky.cpp
    src/linear_solvers/quotient_graph.cpp
    src/linear_solvers/sparse_block_linear_solver_cholesky_emd.cpp
    src/linear_solvers/sparse_block_linear_solver_cholesky_diagonal.cpp
    src/linear_solvers/sparse_block_linear_solver_cholesky_csparse.cpp
    src/linear_solvers/sparse_block_linear_solver_cholesky_cholmod.cpp
    src/linear_solvers/sparse_block_linear_solver_cholmod_full.cpp
    src/linear_solvers/sparse_block_linear_solver_ldl.cpp
    src/linear_solvers/instances.cpp
)
target_include_directories(
  srrg2_solver_linear_solvers
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(
  srrg2_solver_linear_solvers
  PUBLIC
    wombat_srrg::srrg2_solver::sparse_block_matrix
    wombat_srrg::srrg2_core::config
    #${CHOLMOD_LIBRARIES}
    #${UMFPACK_LIBRARIES}
    #${CXSPARSE_LIBRARY}
)
ament_target_dependencies(srrg2_solver_linear_solvers PUBLIC CHOLMOD UMFPACK CXSPARSE)

###########
## core library
############
add_library(srrg2_solver_core SHARED)
add_library(wombat_srrg::srrg2_solver::core ALIAS srrg2_solver_core)

target_sources(
  srrg2_solver_core
  PRIVATE
    #src/solver_action_draw.cpp
    src/solver_stats.cpp
    src/termination_criteria.cpp
    src/iteration_algorithm_base.cpp
    src/iteration_algorithm_gn.cpp
    src/iteration_algorithm_lm.cpp
    src/iteration_algorithm_dl.cpp
    src/iteration_algorithm_ddl.cpp
    src/iteration_algorithm_gn_ls.cpp
    src/iteration_algorithm_gn_pd.cpp
    src/robustifier.cpp
    src/robustifier_policy.cpp
    src/factor_base.cpp
    src/factor_graph.cpp
    src/factor_graph_interface.cpp
    src/factor_graph_view.cpp
    src/solver.cpp
    src/solver_action_base.cpp
    src/instances.cpp
)
target_include_directories(
  srrg2_solver_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(
  srrg2_solver_core
  PUBLIC
  wombat_srrg::srrg2_solver::linear_solvers
  wombat_srrg::srrg2_core::system_utils
  wombat_srrg::srrg2_core::property
  wombat_srrg::srrg2_core::config
  wombat_srrg::srrg2_core::data_structures
  #srrg2_viewer_library
)
