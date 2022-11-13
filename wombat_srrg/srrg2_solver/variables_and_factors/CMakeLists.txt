#add_subdirectory(types_2d)
#add_subdirectory(types_3d)
#add_subdirectory(types_projective)
#add_subdirectory(types_calib)


###########
## types_common library
############
add_library(srrg2_solver_types_common SHARED)
add_library(wombat_srrg::srrg2_solver::types_common ALIAS srrg2_solver_types_common)
target_sources(
  srrg2_solver_types_common
  PRIVATE
    src/types_common/instances.cpp
)
target_include_directories(
  srrg2_solver_types_common
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(
  srrg2_solver_types_common
  PUBLIC
    wombat_srrg::srrg2_solver::core
)
