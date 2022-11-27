find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_measurement_adaptor test_measurement_adaptor.cpp)
target_link_libraries(
  test_measurement_adaptor
    wombat_srrg::srrg2_core::benchmark
    wombat_srrg::srrg2_core::test
    wombat_srrg::srrg2_laser_slam_2d
)

ament_add_gtest(test_slam test_slam.cpp)
target_link_libraries(
  test_slam
    wombat_srrg::srrg2_core::benchmark
    wombat_srrg::srrg2_core::test
    wombat_srrg::srrg2_laser_slam_2d
)
target_compile_definitions(test_slam PUBLIC CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
