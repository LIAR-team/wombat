find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_sim_time_manager test_sim_time_manager.cpp)
target_link_libraries(test_sim_time_manager kennel::kennel_lib rclcpp::rclcpp)

ament_add_gtest(test_kennel_api test_kennel_api.cpp
  ENV KENNEL_TEST_DATADIR=${CMAKE_CURRENT_SOURCE_DIR}/data)
target_link_libraries(test_kennel_api kennel::kennel_lib rclcpp::rclcpp)

ament_add_gtest(test_empty_world test_empty_world.cpp
  ENV KENNEL_TEST_DATADIR=${CMAKE_CURRENT_SOURCE_DIR}/data)
target_link_libraries(test_empty_world kennel::kennel_lib rclcpp::rclcpp)
