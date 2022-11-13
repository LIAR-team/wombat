###########
## utils library
############
add_library(srrg2_solver_utils SHARED)
add_library(wombat_srrg::srrg2_solver::utils ALIAS srrg2_solver_utils)
target_sources(
  srrg2_solver_utils
  PRIVATE
    src/instances.cpp

    #visit
    src/factor_graph_visit_entry.cpp
    src/factor_graph_visit_cost.cpp
    src/factor_graph_visit_policy.cpp
    src/factor_graph_visit.cpp

    # initializer
    src/factor_graph_initializer_rule.cpp
    src/factor_graph_initializer.cpp

    # closure validator
    src/factor_graph_closure_validator.cpp

    src/solver_evaluator.cpp
)
target_include_directories(
  srrg2_solver_utils
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(
  srrg2_solver_utils
  PUBLIC
    wombat_srrg::srrg2_core::config
    wombat_srrg::srrg2_solver::core
    wombat_srrg::srrg2_solver::types_2d
    wombat_srrg::srrg2_solver::types_3d
    wombat_srrg::srrg2_solver::types_projective
    wombat_srrg::srrg2_solver::types_calib
)
