
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
    ${CHOLMOD_LIBRARIES}
    ${UMFPACK_LIBRARIES}
    ${CSPARSE_LIBRARY}
)



return()

add_library(srrg2_solver_core_library SHARED
  #common
  solver_action_base.h

  solver_action_draw.cpp
  solver_action_draw.h

  solver_stats.h
  solver_stats.cpp

  termination_criteria.h
  termination_criteria.cpp

  iteration_algorithm_base.h
  iteration_algorithm_base.cpp

  iteration_algorithm_gn.h
  iteration_algorithm_gn.cpp

  iteration_algorithm_lm.h
  iteration_algorithm_lm.cpp

  iteration_algorithm_dl.h
  iteration_algorithm_dl.cpp

  iteration_algorithm_ddl.h
  iteration_algorithm_ddl.cpp
  
  iteration_algorithm_gn_ls.h
  iteration_algorithm_gn_ls.cpp
  
  iteration_algorithm_gn_pd.h
  iteration_algorithm_gn_pd.cpp

  robustifier.h
  robustifier.cpp

  robustifier_policy.h
  robustifier_policy.cpp
  
  variable.h
  ad_variable.h

  factor_base.h
  factor_base.cpp

  factor.h

  factor_graph.h
  factor_graph.cpp
  factor_graph_interface.cpp
  factor_graph_view.cpp
  
  variable_ptr_tuple.h
  ad_error_factor.h
  ad_constraint_factor.h
  solver.h
  solver.cpp
  solver_action_base.h
  solver_action_base.cpp

  
  instances.h
  instances.cpp
  )

target_link_libraries(srrg2_solver_core_library
  srrg2_solver_linear_solvers_library
  srrg2_system_utils_library
  srrg2_property_library
  srrg2_config_library
  srrg2_data_structures_library
  #ia potential gui stuff
  srrg2_viewer_library
  )

