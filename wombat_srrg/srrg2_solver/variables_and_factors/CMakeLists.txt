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

###########
## types_2d library
############
add_library(srrg2_solver_types_2d SHARED)
add_library(wombat_srrg::srrg2_solver::types_2d ALIAS srrg2_solver_types_2d)
target_sources(
  srrg2_solver_types_2d
  PRIVATE
    src/types_2d/instances.cpp
    src/types_2d/variable_se2.cpp
    src/types_2d/variable_se2_ad.cpp  
)
target_include_directories(
  srrg2_solver_types_2d
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(
  srrg2_solver_types_2d
  PUBLIC
    wombat_srrg::srrg2_core::pcl
    wombat_srrg::srrg2_solver::core
    wombat_srrg::srrg2_solver::types_common
)

###########
## types_3d library
############
add_library(srrg2_solver_types_3d SHARED)
add_library(wombat_srrg::srrg2_solver::types_3d ALIAS srrg2_solver_types_3d)
target_sources(
  srrg2_solver_types_3d
  PRIVATE
    src/types_3d/instances.cpp
    # variable stuff
    src/types_3d/variable_matchable.cpp
    src/types_3d/variable_se3.cpp
    src/types_3d/variable_se3_ad.cpp
    src/types_3d/imu_bias_variable.cpp

    # tg correspondence driven factors
    src/types_3d/se3_plane2plane_error_factor.cpp
    src/types_3d/se3_point2point_error_factor.cpp
    src/types_3d/se3_matchable2matchable_error_factor.cpp
    
    # tg pose-landmark factors 
    src/types_3d/se3_pose_matchable_error_factor.cpp
    src/types_3d/se3_pose_point_offset_error_factor.cpp
    src/types_3d/se3_pose_point_error_factor.cpp
    
    # tg prior factors
    src/types_3d/se3_prior_error_factor_ad.cpp
    src/types_3d/se3_prior_offset_error_factor_ad.cpp
    # tg pose-pose factors
    src/types_3d/se3_pose_pose_geodesic_error_factor.cpp
    src/types_3d/se3_pose_pose_chordal_error_factor.cpp
    src/types_3d/se3_pose_pose_chordal_hessian_factor.cpp
    # tg pose-pose factor
    src/types_3d/se3_pose_pose_right_chordal_error_factor.cpp

    # ldg imu ang gps factor stuff 
    src/types_3d/preintegrated_imu_measurements.cpp
    src/types_3d/imu_factor_ad.cpp
    src/types_3d/gps_factor_ad.cpp
)
target_include_directories(
  srrg2_solver_types_3d
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(
  srrg2_solver_types_3d
  PUBLIC
    wombat_srrg::srrg2_core::matchable
    wombat_srrg::srrg2_solver::core
    wombat_srrg::srrg2_solver::types_common
)
