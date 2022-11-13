find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_linear_solver_block_inverse test_linear_solver_block_inverse.cpp)
target_link_libraries(
  test_linear_solver_block_inverse
  wombat_srrg::srrg2_core::boss
  wombat_srrg::srrg2_core::property
  wombat_srrg::srrg2_solver::core
)

# 2D icp test
ament_add_gtest(test_se2_icp test_se2_icp.cpp)
target_link_libraries(
  test_se2_icp
  wombat_srrg::srrg2_solver::types_2d
  wombat_srrg::srrg2_solver::core
)

# 2D n(ormal)-icp test
ament_add_gtest(test_se2_nicp test_se2_nicp.cpp)
target_link_libraries(
  test_se2_nicp
  wombat_srrg::srrg2_solver::types_2d
  wombat_srrg::srrg2_solver::core
)

# prior factors SE2
ament_add_gtest(test_se2_prior_factor test_se2_prior_factor.cpp)
target_link_libraries(
  test_se2_prior_factor
  wombat_srrg::srrg2_solver::types_2d
  wombat_srrg::srrg2_solver::core
)

# pose pose offset with se2 variable
ament_add_gtest(test_se2_extrinsic_motion_calib test_se2_extrinsic_motion_calib.cpp)
target_link_libraries(
  test_se2_extrinsic_motion_calib
  wombat_srrg::srrg2_solver::core
  wombat_srrg::srrg2_solver::types_calib
  wombat_srrg::srrg2_solver::types_2d
  wombat_srrg::srrg2_solver::types_3d
)

# test pose pose offset with se3 variable
ament_add_gtest(test_se3_extrinsic_motion_calib test_se3_extrinsic_motion_calib.cpp)
target_link_libraries(
  test_se3_extrinsic_motion_calib
  wombat_srrg::srrg2_solver::core
  wombat_srrg::srrg2_solver::types_calib
  wombat_srrg::srrg2_solver::types_2d
  wombat_srrg::srrg2_solver::types_3d
)

# prior factors SE3
ament_add_gtest(test_se3_prior_factor test_se3_prior_factor.cpp)
target_link_libraries(
  test_se3_prior_factor
  wombat_srrg::srrg2_solver::core
  wombat_srrg::srrg2_solver::types_3d
)

# 3D n(ormal)-icp test
ament_add_gtest(test_se3_nicp test_se3_nicp.cpp)
target_link_libraries(
  test_se3_nicp
  wombat_srrg::srrg2_solver::core
  wombat_srrg::srrg2_solver::types_3d
)

# 3D icp test
ament_add_gtest(test_se3_icp test_se3_icp.cpp)
target_link_libraries(
  test_se3_icp
  wombat_srrg::srrg2_solver::core
  wombat_srrg::srrg2_solver::types_3d
)

# 3D motion-based-calibration test
ament_add_gtest(test_se3_motion_based_calib test_se3_motion_based_calib.cpp)
target_link_libraries(
  test_se3_motion_based_calib
  wombat_srrg::srrg2_solver::core
  wombat_srrg::srrg2_solver::types_3d
)

# pose point offset test
ament_add_gtest(test_se3_multi_point_registration test_se3_multi_point_registration.cpp)
target_link_libraries(
  test_se3_multi_point_registration
  wombat_srrg::srrg2_solver::core
  wombat_srrg::srrg2_solver::types_3d
)

# 3D pgo test
ament_add_gtest(test_se3_pgo test_se3_pgo.cpp)
target_link_libraries(
  test_se3_pgo
  wombat_srrg::srrg2_solver::core
  wombat_srrg::srrg2_solver::types_3d
)

# test cholmod linear solver on se3 pgo problems
ament_add_gtest(test_sparse_block_linear_solver_cholmod_full test_sparse_block_linear_solver_cholmod_full.cpp)
target_link_libraries(
  test_sparse_block_linear_solver_cholmod_full
  wombat_srrg::srrg2_solver::core
  wombat_srrg::srrg2_solver::types_3d
)

# se2 pgo test using pose pose chordal error factor (no AD)
ament_add_gtest(test_se2_pgo test_se2_pgo.cpp)
target_link_libraries(
  test_se2_pgo
  wombat_srrg::srrg2_solver::core
  wombat_srrg::srrg2_solver::types_2d
)

# se2 pose point factor test
ament_add_gtest(test_se2_multi_point_registration test_se2_multi_point_registration.cpp)
target_link_libraries(
  test_se2_multi_point_registration
  wombat_srrg::srrg2_solver::core
  wombat_srrg::srrg2_solver::types_2d
)

# 3D pgo test
ament_add_gtest(test_se3_matchables test_se3_matchables.cpp)
target_link_libraries(
  test_se3_matchables
  wombat_srrg::srrg2_solver::core
  wombat_srrg::srrg2_solver::types_3d
)

# g2o converter test
#ament_add_gtest(test_g2o_graph_converter test_g2o_graph_converter.cpp)
#target_link_libraries(
#  test_g2o_graph_converter
#  wombat_srrg::srrg2_solver::core
#  wombat_srrg::srrg2_solver::types_3d
#  srrg2_g2o_converter_library
#  ${SOLVER_EXAMPLES_CORE_LIBRARIES}
#  ${catkin_LIBRARIES}
#)

ament_add_gtest(factor_graph_loop_validator_test factor_graph_loop_validator_test.cpp)
target_link_libraries(
  factor_graph_loop_validator_test
  wombat_srrg::srrg2_core::boss
  wombat_srrg::srrg2_core::property
  wombat_srrg::srrg2_solver::core
  wombat_srrg::srrg2_solver::types_2d
  wombat_srrg::srrg2_solver::types_3d
  wombat_srrg::srrg2_solver::utils
)

# factors in new format
ament_add_gtest(test_se3_posit test_se3_posit.cpp)
target_link_libraries(
  test_se3_posit
  wombat_srrg::srrg2_core::srrg_test
  wombat_srrg::srrg2_solver::types_projective
)

# test similiarity variable
ament_add_gtest(test_sim3 test_sim3.cpp)
target_link_libraries(
  test_sim3
  wombat_srrg::srrg2_solver::core
  wombat_srrg::srrg2_solver::types_projective
)

# test pgo with sim3 variable
ament_add_gtest(test_sim3_pgo test_sim3_pgo.cpp)
target_link_libraries(
  test_sim3_pgo
  wombat_srrg::srrg2_solver::core
  wombat_srrg::srrg2_solver::types_projective
)

# imu test
#catkin_add_gtest(test_imu test_imu.cpp)
#target_link_libraries(test_imu
#   srrg2_solver_core_library
#   srrg2_solver_types3d_library
#   ${catkin_LIBRARIES}
#)

# test icp with sim3 variable
# catkin_add_gtest(test_sim3_icp test_sim3_icp.cpp)
# target_link_libraries(test_sim3_icp
#   srrg2_solver_core_library
#   srrg2_solver_projective_library
#   ${catkin_LIBRARIES}
# )
