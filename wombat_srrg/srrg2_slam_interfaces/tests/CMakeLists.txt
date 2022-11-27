find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_motion_model test_motion_model.cpp)
target_link_libraries(
  test_motion_model
  wombat_srrg::srrg2_slam_interfaces
  wombat_srrg::srrg2_core::test
)

ament_add_gtest(test_motion_model_slice test_motion_model_slice.cpp)
target_link_libraries(
  test_motion_model_slice
  wombat_srrg::srrg2_slam_interfaces
  wombat_srrg::srrg2_core::test
)

ament_add_gtest(test_local_map_splitting_criterion test_local_map_splitting_criterion.cpp)
target_link_libraries(
  test_local_map_splitting_criterion
  wombat_srrg::srrg2_slam_interfaces
  wombat_srrg::srrg2_core::test
)
